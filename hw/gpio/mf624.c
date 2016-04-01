/*
 * Humusoft MF624 DAQ card implementation
 *
 * Copyright (C) 2011 Rostislav Lisovy (lisovy@gmail.com)
 *
 * Licensed under GPLv2 license
 */
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "qemu/event_notifier.h"
#include "qemu/osdep.h"
#include "qemu/thread.h"
#include "qemu/sockets.h"
#include "sysemu/char.h"

#define TYPE_MF624_DEV "mf624"

#define MF624_DEV(obj) \
    OBJECT_CHECK(mf624_state_t, (obj), TYPE_MF624_DEV)

#define PCI_VENDOR_ID_HUMUSOFT        0x186c
#define PCI_DEVICE_ID_MF624        0x0624
#define PCI_CLASS_SIGNAL_PROCESSING_CONTROLLER     0x1180


#define BAR0_size        32
#define BAR2_size        128
#define BAR4_size        128

/* BAR0 */
#define INTCSR_off       0x4C
#define GPIOC_off        0x54

/* BAR2 */
#define ADDATA_off       0x00
#define ADCTRL_off       0x00
#define ADDATA1_off      0x02
#define ADDATA2_off      0x04
#define ADDATA3_off      0x06
#define ADDATA4_off      0x08
#define ADDATA5_off      0x0A
#define ADDATA6_off      0x0C
#define ADDATA7_off      0x0E
#define DOUT_off         0x10
#define DIN_off          0x10
#define ADSTART_off      0x20
#define DA0_off          0x20
#define DA1_off          0x22
#define DA2_off          0x24
#define DA3_off          0x26
#define DA4_off          0x28
#define DA5_off          0x2A
#define DA6_off          0x2C
#define DA7_off          0x2E

#define GPIOC_EOLC_mask  (1 << 17)
#define GPIOC_LDAC_mask  (1 << 23)
#define GPIOC_DACEN_mask (1 << 26)


typedef struct {
    uint32_t INTCSR;
    uint32_t GPIOC;
} BAR0_t;

typedef struct {
    uint16_t ADDATA;
    uint16_t ADCTRL;
    uint16_t ADDATA1;
    uint16_t ADDATA2;
    uint16_t ADDATA3;
    uint16_t ADDATA4;
    uint16_t ADDATA5;
    uint16_t ADDATA6;
    uint16_t ADDATA7;
    uint16_t DIN;
    uint16_t DOUT;
    uint16_t DA0;
    uint16_t DA1;
    uint16_t DA2;
    uint16_t DA3;
    uint16_t DA4;
    uint16_t DA5;
    uint16_t DA6;
    uint16_t DA7;
} BAR2_t;

/* Not implemented */
typedef struct {
//    uint32_t CTR0STATUS;
//    uint32_t CTR0MODE;
//    uint32_t CTR0;
//    uint32_t CTR0A;
//     ...
} BAR4_t;

typedef struct {
    PCIDevice dev;
    MemoryRegion mmio_bar0;
    MemoryRegion mmio_bar2;
    MemoryRegion mmio_bar4;
        qemu_irq     irq;

        QemuThread socket_thread;
    int socket_srv;
    int socket_tmp;
    uint32_t port;
    int addr;

    /* The real voltage which is on inputs od A/D convertors.
    Until new conversion is started, there is still old value in ADC registers*/
    unsigned int real_world_AD0; /* Value in "ADC internal" format */
    unsigned int real_world_AD1;
    unsigned int real_world_AD2;
    unsigned int real_world_AD3;
    unsigned int real_world_AD4;
    unsigned int real_world_AD5;
    unsigned int real_world_AD6;
    unsigned int real_world_AD7;

    /* for cpu_register_physical_memory() function */
    unsigned int BAR0_mem_table_index;
    unsigned int BAR2_mem_table_index;
    unsigned int BAR4_mem_table_index;

    /* Internal registers values */
    BAR0_t BAR0;
    BAR2_t BAR2;
    BAR4_t BAR4;


    int ADDATA_FIFO[8]; /* this array tells us which ADCs are being converted */
    unsigned int ADDATA_FIFO_POSITION; /* ADDATA is FIFO register;
                 * We need to know, position in this FIFO =
                 * Which value will come next */
} mf624_state_t;

int mf624_instance; /* Global variable shared between multiple mf624 devices */


static int16_t volts_to_adinternal(double volt)
{
    if (volt > 9.99) {
        volt = 9.99;
    } else if (volt < -10) {
        volt = -10;
    }

    return ((int16_t) ((volt*0x8000)/10))>>2;
}

static double dacinternal_to_volts(int16_t dacinternal)
{
    return (((double)dacinternal)/0x4000)*20.0 - 10.0;
}

/*----------------------------------------------------------------------------*/

/* Initialize register values due to MF624 manual */
static void mf624_init_registers(mf624_state_t *s)
{
#define INTCSR_default_value    0x000300
#define GPIOC_default_value     (0x006C0 | (0x10 << 21) | (2 << 25))

    /* Initialize all registers to default values */
    s->BAR0.INTCSR = INTCSR_default_value;
    s->BAR0.GPIOC = GPIOC_default_value;
    s->BAR2.ADDATA = 0x0;
    s->BAR2.ADCTRL = 0x0;
    s->BAR2.ADDATA1 = 0x0;
    s->BAR2.ADDATA2 = 0x0;
    s->BAR2.ADDATA3 = 0x0;
    s->BAR2.ADDATA4 = 0x0;
    s->BAR2.ADDATA5 = 0x0;
    s->BAR2.ADDATA6 = 0x0;
    s->BAR2.ADDATA7 = 0x0;

    s->BAR2.DIN = 0xFF;
    s->BAR2.DOUT = 0x00;
    s->BAR2.DA0 = 0x3FFF;
    s->BAR2.DA1 = 0x3FFF;
    s->BAR2.DA2 = 0x3FFF;
    s->BAR2.DA3 = 0x3FFF;
    s->BAR2.DA4 = 0x3FFF;
    s->BAR2.DA5 = 0x3FFF;
    s->BAR2.DA6 = 0x3FFF;
    s->BAR2.DA7 = 0x3FFF;

    s->ADDATA_FIFO_POSITION = 0;
}

static void
mf624_reset(void *opaque)
{
    mf624_state_t *s = (mf624_state_t *)opaque;

    mf624_init_registers(s);
}

/* After some widget's value is changed, new value is send via socket to QEMU */
static void socket_write(mf624_state_t *s, const char* reg, double val)
{
    int status;
    char write_buffer[256];
    snprintf(write_buffer, 255, "%s=%f\n", reg, val);

    status = write(s->socket_tmp, write_buffer, strlen(write_buffer));
    if (status < 0) {
        /* perror("write()"); */
        printf("Error writing into socket. Is there no client connected?\n");
    }
}

#define STRING_BUFF_SIZE    256
static void socket_read(mf624_state_t *dev)
{
    /* For parsing read instructions */
    char reg[STRING_BUFF_SIZE+1];
    float val;
    /* For reading from socket */
    char read_buffer[STRING_BUFF_SIZE];
    int received_length = 0;
    int status;


    while (1) {
        memset(read_buffer, '\0', STRING_BUFF_SIZE);

        received_length = read(dev->socket_tmp, read_buffer,
                               STRING_BUFF_SIZE-1);

        if (received_length < 0) {
            perror("read()");
            return;
        }

        if (received_length == 0) {
            printf("Error while reading from socket. Client disconnected?\n");
            return;
        }

        /* REG has "same size +1" as READ_BUFFER to avoid buffer overflow */
        status = sscanf(read_buffer, "%[A-Z0-9]=%f", reg, &val);
        if (status == 2) {
            if (!strcmp(reg, "DIN")) {
                dev->BAR2.DIN = val;
            } else if (!strcmp(reg, "ADC0")) {
                dev->real_world_AD0 = volts_to_adinternal(val);
            } else if (!strcmp(reg, "ADC1")) {
                dev->real_world_AD1 = volts_to_adinternal(val);
            } else if (!strcmp(reg, "ADC2")) {
                dev->real_world_AD2 = volts_to_adinternal(val);
            } else if (!strcmp(reg, "ADC3")) {
                dev->real_world_AD3 = volts_to_adinternal(val);
            } else if (!strcmp(reg, "ADC4")) {
                dev->real_world_AD4 = volts_to_adinternal(val);
            } else if (!strcmp(reg, "ADC5")) {
                dev->real_world_AD5 = volts_to_adinternal(val);
            } else if (!strcmp(reg, "ADC6")) {
                dev->real_world_AD6 = volts_to_adinternal(val);
            } else if (!strcmp(reg, "ADC7")) {
                dev->real_world_AD7 = volts_to_adinternal(val);
            } else {
                printf("reg = %s; val = %f\n", reg, val);
            }
        }
    }
}


static void *init_socket(void * ptr)
{
    struct sockaddr_in addr_client;
    struct sockaddr_in addr_srv;
    int port;
    int yes = 1;

    mf624_state_t *dev = (mf624_state_t *) ptr;

    dev->socket_tmp = -1;
    port = dev->port;

    dev->socket_srv = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (dev->socket_srv == -1) {
        perror("socket()");
        return NULL;
    }

    if (setsockopt(dev->socket_srv,
                   SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) {
        perror("setsockopt()");
        return NULL;
    }


    socklen_t len = sizeof(addr_srv);
    memset(&addr_srv, 0, len);
    addr_srv.sin_family = AF_INET;
    addr_srv.sin_addr.s_addr = htonl(INADDR_ANY);
    addr_srv.sin_port = htons(port);
    if (bind(dev->socket_srv, (struct sockaddr *) &addr_srv, len) == -1) {
        perror("bind()");
        return NULL;
    }

    if (listen(dev->socket_srv, 5) == -1) {
        perror("listen()");
        return NULL;
    }


    while (1) {
        printf("Waiting on port %d for MF624 client to connect\n", dev->port);
        socklen_t len_client = sizeof(addr_client);
        dev->socket_tmp = accept(dev->socket_srv,
                                 (struct sockaddr *) &addr_client, &len_client);
        if (dev->socket_tmp == -1) {
            perror("accept()");
                        return NULL;
        }

        printf("Client connected\n");

        socket_read(dev); /* should run forever if everything is OK; */
                       /* If error occurs (client disconnected), returns here */

        close(dev->socket_tmp);
    }

    return NULL;
}

/*----------------------------------------------------------------------------*/

static void mf624_BAR0_write32(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    mf624_state_t *s = opaque;

    switch (addr % BAR0_size) {
    case INTCSR_off:
        s->BAR0.INTCSR = (value & 0x7FF) | INTCSR_default_value; /* Only first 11 bits are writable */
        socket_write(s, "INTCSR", s->BAR0.INTCSR);
        break;

    case GPIOC_off:
        /* Don't write anywhere else than into these two bits */
        s->BAR0.GPIOC = (value & (GPIOC_LDAC_mask | GPIOC_DACEN_mask)) |
                         GPIOC_default_value;

        socket_write(s, "GPIOC", s->BAR0.GPIOC);

        /* Is DAC enabled & Output enabled? */
        if (!(s->BAR0.GPIOC & GPIOC_LDAC_mask) &&
            (s->BAR0.GPIOC & GPIOC_DACEN_mask)) {
            socket_write(s, "DA0", dacinternal_to_volts(s->BAR2.DA0));
            socket_write(s, "DA1", dacinternal_to_volts(s->BAR2.DA1));
            socket_write(s, "DA2", dacinternal_to_volts(s->BAR2.DA2));
            socket_write(s, "DA3", dacinternal_to_volts(s->BAR2.DA3));
            socket_write(s, "DA4", dacinternal_to_volts(s->BAR2.DA4));
            socket_write(s, "DA5", dacinternal_to_volts(s->BAR2.DA5));
            socket_write(s, "DA6", dacinternal_to_volts(s->BAR2.DA6));
            socket_write(s, "DA7", dacinternal_to_volts(s->BAR2.DA7));
        }

        /* Is output forced to GND? */
        if (!(s->BAR0.GPIOC & GPIOC_DACEN_mask)) {
            #define GND     0
            socket_write(s, "DA0", dacinternal_to_volts(GND));
            socket_write(s, "DA1", dacinternal_to_volts(GND));
            socket_write(s, "DA2", dacinternal_to_volts(GND));
            socket_write(s, "DA3", dacinternal_to_volts(GND));
            socket_write(s, "DA4", dacinternal_to_volts(GND));
            socket_write(s, "DA5", dacinternal_to_volts(GND));
            socket_write(s, "DA6", dacinternal_to_volts(GND));
            socket_write(s, "DA7", dacinternal_to_volts(GND));
        }
        break;

    default:
        printf("mf624_BAR0_write32(): addr = " TARGET_FMT_plx
                           "; value = 0x%" PRIx64 "\n", addr, value);
        break;
    }
}

static uint64_t mf624_BAR0_read32(void *opaque, hwaddr addr, unsigned size)
{
    mf624_state_t *s = opaque;

    switch (addr % BAR0_size) {
    case INTCSR_off:
        return s->BAR0.INTCSR;

    case GPIOC_off:
        return s->BAR0.GPIOC;

    default:
        printf("mf624_BAR0_read32(): addr = "
                           TARGET_FMT_plx "\n", addr);
        return 0x0;
    }
}

static uint64_t mf624_BAR2_read16(void *opaque, hwaddr addr, unsigned size)
{
    int i;
    int ADDATA_val = 0xFFFF;
    mf624_state_t *s = opaque;

    switch (addr % BAR2_size) {
    /* Reading from ADDATA FIFO register */
    case ADDATA_off:
    case ADDATA1_off: /* Mirrored registers */
    case ADDATA2_off:
    case ADDATA3_off:
    case ADDATA4_off:
    case ADDATA5_off:
    case ADDATA6_off:
    case ADDATA7_off:
        if (!(s->BAR0.GPIOC & GPIOC_EOLC_mask)) { /* Has the conversion already ended? */
            #define ADC_CHANNELS    8
            for (i = s->ADDATA_FIFO_POSITION; i < ADC_CHANNELS; i++) {
                if (s->BAR2.ADCTRL & (1 << i)) {
                    s->ADDATA_FIFO_POSITION = i; /* Move to next AD to be read */
                }
            }

            switch (s->ADDATA_FIFO_POSITION) {
            case 0:
                ADDATA_val = s->BAR2.ADDATA;
                break;
            case 1:
                ADDATA_val = s->BAR2.ADDATA1;
                break;
            case 2:
                ADDATA_val = s->BAR2.ADDATA2;
                break;
            case 3:
                ADDATA_val = s->BAR2.ADDATA3;
                break;
            case 4:
                ADDATA_val = s->BAR2.ADDATA4;
                break;
            case 5:
                ADDATA_val = s->BAR2.ADDATA5;
                break;
            case 6:
                ADDATA_val = s->BAR2.ADDATA6;
                break;
            case 7:
                ADDATA_val = s->BAR2.ADDATA7;
                break;
            default: /* restart counter */
                s->ADDATA_FIFO_POSITION = 0;
                ADDATA_val = s->BAR2.ADDATA;
                break;
            }
            s->ADDATA_FIFO_POSITION++;
            return ADDATA_val;
        }
        return 0xFFFF; /* Semirandom value */

    /* Digital Input*/
    case DIN_off:
        return s->BAR2.DIN;

    /* A/D Conversion Start. Reading this register triggers A/D
    conversion for all channels selected in ADCTRL. */
    case ADSTART_off:
        s->BAR0.GPIOC |= GPIOC_EOLC_mask; /* Conversion in progress */
        s->ADDATA_FIFO_POSITION = 0;

        /* Simulation of the time delay of real conversion should be implemented there */

        /* Check before assignement, if particular ADC is enabled */
        s->BAR2.ADDATA  = (s->BAR2.ADCTRL & (1 << 0)) ?
                              s->real_world_AD0 : s->BAR2.ADDATA;
        s->BAR2.ADDATA1 = (s->BAR2.ADCTRL & (1 << 1)) ?
                              s->real_world_AD1 : s->BAR2.ADDATA1;
        s->BAR2.ADDATA2 = (s->BAR2.ADCTRL & (1 << 2)) ?
                              s->real_world_AD2 : s->BAR2.ADDATA2;
        s->BAR2.ADDATA3 = (s->BAR2.ADCTRL & (1 << 3)) ?
                              s->real_world_AD3 : s->BAR2.ADDATA3;
        s->BAR2.ADDATA4 = (s->BAR2.ADCTRL & (1 << 4)) ?
                              s->real_world_AD4 : s->BAR2.ADDATA4;
        s->BAR2.ADDATA5 = (s->BAR2.ADCTRL & (1 << 5)) ?
                              s->real_world_AD5 : s->BAR2.ADDATA5;
        s->BAR2.ADDATA6 = (s->BAR2.ADCTRL & (1 << 6)) ?
                              s->real_world_AD6 : s->BAR2.ADDATA6;
        s->BAR2.ADDATA7 = (s->BAR2.ADCTRL & (1 << 7)) ?
                              s->real_world_AD7 : s->BAR2.ADDATA7;

        /* All channels converted */
        s->BAR0.GPIOC &= ~GPIOC_EOLC_mask;

        return 0xFFFF; /* Semirandom value */

    default:
        printf("mf624_BAR2_read16(): addr = "
                           TARGET_FMT_plx "\n", addr);
        return 0x0;
    }
}

static void mf624_BAR2_write16(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    mf624_state_t *s = opaque;

    switch (addr % BAR2_size) {
    case ADCTRL_off:
        s->BAR2.ADCTRL = value;
        socket_write(s, "ADCTRL", s->BAR2.ADCTRL);
        break;

    case DOUT_off:
        s->BAR2.DOUT = value;
        socket_write(s, "DOUT", s->BAR2.DOUT);
        break;

    case DA0_off:
        s->BAR2.DA0 = value;
        /* Is DAC enabled & Output enabled? */
        if (!(s->BAR0.GPIOC & GPIOC_LDAC_mask) &&
            (s->BAR0.GPIOC & GPIOC_DACEN_mask)) {
            socket_write(s, "DA0", dacinternal_to_volts(s->BAR2.DA0));
        }
        break;

    case DA1_off:
        s->BAR2.DA1 = value;
        if (!(s->BAR0.GPIOC & GPIOC_LDAC_mask) &&
            (s->BAR0.GPIOC & GPIOC_DACEN_mask)) {
            socket_write(s, "DA1", dacinternal_to_volts(s->BAR2.DA1));
        }
        break;

    case DA2_off:
        s->BAR2.DA2 = value;
        if (!(s->BAR0.GPIOC & GPIOC_LDAC_mask) &&
            (s->BAR0.GPIOC & GPIOC_DACEN_mask)) {
            socket_write(s, "DA2", dacinternal_to_volts(s->BAR2.DA2));
        }
        break;

    case DA3_off:
        s->BAR2.DA3 = value;
        if (!(s->BAR0.GPIOC & GPIOC_LDAC_mask) &&
            (s->BAR0.GPIOC & GPIOC_DACEN_mask)) {
            socket_write(s, "DA3", dacinternal_to_volts(s->BAR2.DA3));
        }
        break;

    case DA4_off:
        s->BAR2.DA4 = value;
        if (!(s->BAR0.GPIOC & GPIOC_LDAC_mask) &&
            (s->BAR0.GPIOC & GPIOC_DACEN_mask)) {
            socket_write(s, "DA4", dacinternal_to_volts(s->BAR2.DA4));
        }
        break;

    case DA5_off:
        s->BAR2.DA5 = value;
        if (!(s->BAR0.GPIOC & GPIOC_LDAC_mask) &&
            (s->BAR0.GPIOC & GPIOC_DACEN_mask)) {
            socket_write(s, "DA5", dacinternal_to_volts(s->BAR2.DA5));
        }
        break;

    case DA6_off:
        s->BAR2.DA6 = value;
        if (!(s->BAR0.GPIOC & GPIOC_LDAC_mask) &&
            (s->BAR0.GPIOC & GPIOC_DACEN_mask)) {
            socket_write(s, "DA6", dacinternal_to_volts(s->BAR2.DA6));
        }
        break;

    case DA7_off:
        s->BAR2.DA7 = value;
        if (!(s->BAR0.GPIOC & GPIOC_LDAC_mask) &&
            (s->BAR0.GPIOC & GPIOC_DACEN_mask)) {
            socket_write(s, "DA7", dacinternal_to_volts(s->BAR2.DA7));
        }
        break;

    default:
        printf("mf624_BAR2_write16(): addr = " TARGET_FMT_plx
                           "; value = 0x%" PRIx64 "\n", addr, value);
        break;
    }
}

static void mf624_BAR4_write32(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    printf("mf624_BAR4_write32(): addr = " TARGET_FMT_plx
               "; value = 0x%" PRIx64 "\n", addr, value);
}

static uint64_t mf624_BAR4_read32(void *opaque, hwaddr addr, unsigned size)
{
    printf("mf624_BAR4_read32(): addr = " TARGET_FMT_plx "\n", addr);
    return 0x0;
}

/*----------------------------------------------------------------------------*/

static const MemoryRegionOps mf624_BAR0_mmio_ops = {
    .read = mf624_BAR0_read32,
    .write = mf624_BAR0_write32,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static const MemoryRegionOps mf624_BAR2_mmio_ops = {
    .read = mf624_BAR2_read16,
    .write = mf624_BAR2_write16,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 2,
    },
};

static const MemoryRegionOps mf624_BAR4_mmio_ops = {
    .read = mf624_BAR4_read32,
    .write = mf624_BAR4_write32,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

#define DEFAULT_PORT            55555
static int mf624_init(PCIDevice *pci_dev)
{
    mf624_state_t *s = MF624_DEV(pci_dev); /* alocation of mf624_state_t */
    uint8_t *pci_conf;

    if (s->port == DEFAULT_PORT) {
        s->port += mf624_instance; /* Each instance of the same device should have another port number */
        mf624_instance++;
    }

    /* Set all internal registers to default values */
    mf624_init_registers(s);

    pci_conf = pci_dev->config;
    pci_conf[PCI_INTERRUPT_PIN] = 0x1; /* interrupt pin 0 */

    s->irq = pci_allocate_irq(&s->dev);

    qemu_register_reset(mf624_reset, s);

    memory_region_init_io(&s->mmio_bar0, OBJECT(s), &mf624_BAR0_mmio_ops, s, "mf624_bar0", BAR0_size);
    memory_region_init_io(&s->mmio_bar2, OBJECT(s), &mf624_BAR2_mmio_ops, s, "mf624_bar2", BAR2_size);
    memory_region_init_io(&s->mmio_bar4, OBJECT(s), &mf624_BAR4_mmio_ops, s, "mf624_bar4", BAR4_size);
    pci_register_bar(&s->dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio_bar0);
    pci_register_bar(&s->dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio_bar2);
    pci_register_bar(&s->dev, 4, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio_bar4);

    /* Create thread, which will be blocked on reading from socket (connected to "I/O GUI") */

    qemu_thread_create(&s->socket_thread, "mf624_io_thread",
                       init_socket, (void *) s, QEMU_THREAD_JOINABLE);
    return 0;
}

static void qdev_mf624_reset(DeviceState *dev)
{
    mf624_state_t *s = MF624_DEV(dev);
    mf624_init_registers(s);
}

static void mf624_exit(PCIDevice *pci_dev)
{
    mf624_state_t *s = MF624_DEV(pci_dev);

    close(s->socket_srv);

    qemu_thread_join(&s->socket_thread);

    qemu_unregister_reset(mf624_reset, s);

    /*
     * memory regions s->mmio_bar0, s->mmio_bar2 and s->mmio_bar4
     * are destroyed QOM model automatically
     */
    /* memory_region_destroy(&s->mmio_bar0); */
    /* memory_region_destroy(&s->mmio_bar2); */
    /* memory_region_destroy(&s->mmio_bar4); */

    qemu_free_irq(s->irq);
}

static const VMStateDescription vmstate_mf624 = {
    .name = "mf624",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,

    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(dev, mf624_state_t),
        VMSTATE_UINT32(port, mf624_state_t),
        VMSTATE_END_OF_LIST()
    }

};

static Property mf624_properties[] = {
    DEFINE_PROP_UINT32("port", mf624_state_t, port, DEFAULT_PORT),
    DEFINE_PROP_END_OF_LIST(),
};

static void mf624_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->init = mf624_init;
    k->exit = mf624_exit;
    k->vendor_id = PCI_VENDOR_ID_HUMUSOFT;
    k->device_id = PCI_DEVICE_ID_MF624;
    k->revision = 0x00;
    k->class_id = PCI_CLASS_SIGNAL_PROCESSING_CONTROLLER;
    k->subsystem_vendor_id = PCI_VENDOR_ID_HUMUSOFT;
    k->subsystem_id = PCI_DEVICE_ID_MF624;
    dc->desc = "Humusoft MF624";
    dc->props = mf624_properties;
    dc->vmsd = &vmstate_mf624;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->reset = qdev_mf624_reset;
}

static const TypeInfo mf624_info = {
    .name          = TYPE_MF624_DEV,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(mf624_state_t),
    .class_init    = mf624_class_init,
};

static void mf624_register_types(void)
{
    type_register_static(&mf624_info);
}

type_init(mf624_register_types)
