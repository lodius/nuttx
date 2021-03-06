/* config.h -- Autogenerated! Do not edit. */

#ifndef __INCLUDE_NUTTX_CONFIG_H
#define __INCLUDE_NUTTX_CONFIG_H

/* Architecture-specific options *************************/

#define CONFIG_HOST_LINUX 1
#define CONFIG_APPS_DIR "../apps"
#define CONFIG_BUILD_FLAT 1
#define CONFIG_INTELHEX_BINARY 1
#define CONFIG_RAW_BINARY 1
#define CONFIG_DEBUG 1
#define CONFIG_ARCH_HAVE_HEAPCHECK 1
#define CONFIG_DEBUG_VERBOSE 1
#define CONFIG_ARCH_HAVE_STACKCHECK 1
#define CONFIG_ARCH_HAVE_CUSTOMOPT 1
#define CONFIG_DEBUG_FULLOPT 1
#define CONFIG_ARCH_ARM 1
#define CONFIG_ARCH "arm"
#define CONFIG_ARCH_CHIP_STM32 1
#define CONFIG_ARCH_CORTEXM4 1
#define CONFIG_ARCH_FAMILY "armv7-m"
#define CONFIG_ARCH_CHIP "stm32"
#define CONFIG_ARM_TOOLCHAIN_GNU 1
#define CONFIG_ARCH_HAVE_CMNVECTOR 1
#define CONFIG_ARCH_HAVE_FPU 1
#define CONFIG_ARM_HAVE_MPU_UNIFIED 1
#define CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL 1
#define CONFIG_ARMV7M_HAVE_STACKCHECK 1
#define CONFIG_ARCH_CHIP_STM32F407VG 1
#define CONFIG_STM32_FLASH_CONFIG_DEFAULT 1
#define CONFIG_STM32_STM32F40XX 1
#define CONFIG_STM32_STM32F407 1
#define CONFIG_STM32_HAVE_CCM 1
#define CONFIG_STM32_HAVE_OTGFS 1
#define CONFIG_STM32_HAVE_FSMC 1
#define CONFIG_STM32_HAVE_USART3 1
#define CONFIG_STM32_HAVE_UART4 1
#define CONFIG_STM32_HAVE_UART5 1
#define CONFIG_STM32_HAVE_USART6 1
#define CONFIG_STM32_HAVE_TIM1 1
#define CONFIG_STM32_HAVE_TIM2 1
#define CONFIG_STM32_HAVE_TIM3 1
#define CONFIG_STM32_HAVE_TIM4 1
#define CONFIG_STM32_HAVE_TIM5 1
#define CONFIG_STM32_HAVE_TIM6 1
#define CONFIG_STM32_HAVE_TIM7 1
#define CONFIG_STM32_HAVE_TIM8 1
#define CONFIG_STM32_HAVE_TIM9 1
#define CONFIG_STM32_HAVE_TIM10 1
#define CONFIG_STM32_HAVE_TIM11 1
#define CONFIG_STM32_HAVE_TIM12 1
#define CONFIG_STM32_HAVE_TIM13 1
#define CONFIG_STM32_HAVE_TIM14 1
#define CONFIG_STM32_HAVE_ADC2 1
#define CONFIG_STM32_HAVE_ADC3 1
#define CONFIG_STM32_HAVE_ADC1_DMA 1
#define CONFIG_STM32_HAVE_CAN1 1
#define CONFIG_STM32_HAVE_CAN2 1
#define CONFIG_STM32_HAVE_DAC1 1
#define CONFIG_STM32_HAVE_DAC2 1
#define CONFIG_STM32_HAVE_RNG 1
#define CONFIG_STM32_HAVE_ETHMAC 1
#define CONFIG_STM32_HAVE_I2C2 1
#define CONFIG_STM32_HAVE_I2C3 1
#define CONFIG_STM32_HAVE_SPI2 1
#define CONFIG_STM32_HAVE_SPI3 1
#define CONFIG_STM32_ADC1 1
#define CONFIG_STM32_DMA1 1
#define CONFIG_STM32_DMA2 1
#define CONFIG_STM32_I2C1 1
#define CONFIG_STM32_I2C2 1
#define CONFIG_STM32_PWR 1
#define CONFIG_STM32_SPI1 1
#define CONFIG_STM32_SYSCFG 1
#define CONFIG_STM32_TIM1 1
#define CONFIG_STM32_TIM2 1
#define CONFIG_STM32_TIM4 1
#define CONFIG_STM32_USART2 1
#define CONFIG_STM32_USART6 1
#define CONFIG_STM32_ADC 1
#define CONFIG_STM32_SPI 1
#define CONFIG_STM32_I2C 1
#define CONFIG_STM32_JTAG_SW_ENABLE 1
#define CONFIG_STM32_CCMEXCLUDE 1
#define CONFIG_STM32_DMACAPABLE 1
#define CONFIG_STM32_TIM2_PWM 1
#define CONFIG_STM32_TIM2_MODE 0
#define CONFIG_STM32_TIM2_CHANNEL 1
#define CONFIG_STM32_TIM2_CHMODE 0
#define CONFIG_STM32_TIM4_PWM 1
#define CONFIG_STM32_TIM4_MODE 0
#define CONFIG_STM32_TIM4_CHANNEL 2
#define CONFIG_STM32_TIM4_CHMODE 0
#define CONFIG_STM32_TIM1_ADC 1
#define CONFIG_STM32_TIM1_ADC1 1
#define CONFIG_HAVE_ADC1_TIMER 1
#define CONFIG_STM32_ADC1_SAMPLE_FREQUENCY 100
#define CONFIG_STM32_ADC1_TIMTRIG 0
#define CONFIG_STM32_ADC1_DMA 1
#define CONFIG_STM32_USART 1
#define CONFIG_STM32_FLOWCONTROL_BROKEN 1
#define CONFIG_STM32_SPI_DMA 1
#define CONFIG_STM32_I2CTIMEOSEC 0
#define CONFIG_STM32_I2CTIMEOMS 500
#define CONFIG_STM32_I2CTIMEOTICKS 500
#define CONFIG_RTC_MAGIC_REG 0
#define CONFIG_RTC_MAGIC 0xfacefeee
#define CONFIG_RTC_LSICLOCK 1
#define CONFIG_ARCH_DMA 1
#define CONFIG_ARCH_HAVE_IRQPRIO 1
#define CONFIG_ARCH_HAVE_VFORK 1
#define CONFIG_ARCH_HAVE_MPU 1
#define CONFIG_ARCH_HAVE_RESET 1
#define CONFIG_ARCH_STACKDUMP 1
#define CONFIG_ARCH_HAVE_RAMVECTORS 1
#define CONFIG_BOARD_LOOPSPERMSEC 16717
#define CONFIG_ARCH_HAVE_INTERRUPTSTACK 1
#define CONFIG_ARCH_INTERRUPTSTACK 0
#define CONFIG_ARCH_HAVE_HIPRI_INTERRUPT 1
#define CONFIG_BOOT_RUNFROMFLASH 1
#define CONFIG_RAM_START 0x20000000
#define CONFIG_RAM_SIZE 114688
#define CONFIG_ARCH_BOARD_STM32F4_DISCOVERY 1
#define CONFIG_ARCH_BOARD "stm32f4discovery"
#define CONFIG_ARCH_HAVE_LEDS 1
#define CONFIG_ARCH_LEDS 1
#define CONFIG_ARCH_HAVE_BUTTONS 1
#define CONFIG_ARCH_BUTTONS 1
#define CONFIG_ARCH_HAVE_IRQBUTTONS 1
#define CONFIG_ARCH_IRQBUTTONS 1
#define CONFIG_NSH_MMCSDMINOR 0
#define CONFIG_NSH_MMCSDSLOTNO 0
#define CONFIG_NSH_MMCSDSPIPORTNO 0
#define CONFIG_GSENSOR 1
#define CONFIG_TB6612 1
#define CONFIG_ESP8266 1
#define CONFIG_LIB_BOARDCTL 1
#define CONFIG_BOARDCTL_ADCTEST 1
#define CONFIG_BOARDCTL_PWMTEST 1
#define CONFIG_DISABLE_OS_API 1
#define CONFIG_USEC_PER_TICK 10000
#define CONFIG_SYSTEM_TIME64 1
#define CONFIG_MAX_WDOGPARMS 4
#define CONFIG_PREALLOC_WDOGS 32
#define CONFIG_WDOG_INTRESERVE 4
#define CONFIG_PREALLOC_TIMERS 4
#define CONFIG_INIT_ENTRYPOINT 1
#define CONFIG_USER_ENTRYPOINT nsh_main
#define CONFIG_RR_INTERVAL 200
#define CONFIG_TASK_NAME_SIZE 31
#define CONFIG_MAX_TASKS 16
#define CONFIG_SCHED_WAITPID 1
#define CONFIG_MUTEX_TYPES 1
#define CONFIG_NPTHREAD_KEYS 8
#define CONFIG_DEV_CONSOLE 1
#define CONFIG_SDCLONE_DISABLE 1
#define CONFIG_NFILE_DESCRIPTORS 12
#define CONFIG_NFILE_STREAMS 12
#define CONFIG_NAME_MAX 32
#define CONFIG_SIG_SIGUSR1 1
#define CONFIG_SIG_SIGUSR2 2
#define CONFIG_SIG_SIGALARM 3
#define CONFIG_SIG_SIGCONDTIMEDOUT 16
#define CONFIG_SIG_SIGWORK 17
#define CONFIG_PREALLOC_MQ_MSGS 4
#define CONFIG_MQ_MAXMSGSIZE 32
#define CONFIG_SCHED_WORKQUEUE 1
#define CONFIG_SCHED_HPWORK 1
#define CONFIG_SCHED_HPWORKPRIORITY 224
#define CONFIG_SCHED_HPWORKPERIOD 50000
#define CONFIG_SCHED_HPWORKSTACKSIZE 2048
#define CONFIG_IDLETHREAD_STACKSIZE 2048
#define CONFIG_USERMAIN_STACKSIZE 16384
#define CONFIG_PTHREAD_STACK_MIN 1024
#define CONFIG_PTHREAD_STACK_DEFAULT 16384
#define CONFIG_DEV_NULL 1
#define CONFIG_ARCH_HAVE_PWM_PULSECOUNT 1
#define CONFIG_PWM 1
#define CONFIG_ARCH_HAVE_I2CRESET 1
#define CONFIG_I2C 1
#define CONFIG_I2C_POLLED 1
#define CONFIG_SPI 1
#define CONFIG_TIMER 1
#define CONFIG_RTC 1
#define CONFIG_RTC_DATETIME 1
#define CONFIG_RTC_ALARM 1
#define CONFIG_RTC_NALARMS 1
#define CONFIG_RTC_DRIVER 1
#define CONFIG_RTC_IOCTL 1
#define CONFIG_ANALOG 1
#define CONFIG_ADC 1
#define CONFIG_ADC_FIFOSIZE 16
#define CONFIG_MMCSD 1
#define CONFIG_MMCSD_NSLOTS 1
#define CONFIG_MMCSD_SPI 1
#define CONFIG_MMCSD_SPICLOCK 12000000
#define CONFIG_MMCSD_SPIMODE 0
#define CONFIG_POWER 1
#define CONFIG_SENSORS 1
#define CONFIG_BH1750FVI 1
#define CONFIG_MS58XX_VDD 30
#define CONFIG_I2C_SHT10 1
#define CONFIG_SHT10 1
#define CONFIG_SERIAL 1
#define CONFIG_ARCH_HAVE_USART2 1
#define CONFIG_ARCH_HAVE_USART6 1
#define CONFIG_USART2_ISUART 1
#define CONFIG_USART6_ISUART 1
#define CONFIG_MCU_SERIAL 1
#define CONFIG_STANDARD_SERIAL 1
#define CONFIG_SERIAL_NPOLLWAITERS 2
#define CONFIG_ARCH_HAVE_SERIAL_TERMIOS 1
#define CONFIG_USART2_SERIAL_CONSOLE 1
#define CONFIG_USART2_RXBUFSIZE 128
#define CONFIG_USART2_TXBUFSIZE 128
#define CONFIG_USART2_BAUD 115200
#define CONFIG_USART2_BITS 8
#define CONFIG_USART2_PARITY 0
#define CONFIG_USART2_2STOP 0
#define CONFIG_USART6_RXBUFSIZE 2048
#define CONFIG_USART6_TXBUFSIZE 256
#define CONFIG_USART6_BAUD 115200
#define CONFIG_USART6_BITS 8
#define CONFIG_USART6_PARITY 0
#define CONFIG_USART6_2STOP 0
#define CONFIG_FS_AUTOMOUNTER 1
#define CONFIG_FS_READABLE 1
#define CONFIG_FS_WRITABLE 1
#define CONFIG_FS_MQUEUE_MPATH "/var/mqueue"
#define CONFIG_FS_FAT 1
#define CONFIG_FAT_LCNAMES 1
#define CONFIG_FAT_LFN 1
#define CONFIG_FAT_MAXFNAME 32
#define CONFIG_FS_ROMFS 1
#define CONFIG_FS_PROCFS 1
#define CONFIG_MM_REGIONS 2
#define CONFIG_BUILTIN 1
#define CONFIG_STDIO_BUFFER_SIZE 64
#define CONFIG_STDIO_LINEBUFFER 1
#define CONFIG_NUNGET_CHARS 2
#define CONFIG_LIB_HOMEDIR "/"
#define CONFIG_LIBC_LONG_LONG 1
#define CONFIG_LIB_RAND_ORDER 1
#define CONFIG_EOL_IS_EITHER_CRLF 1
#define CONFIG_POSIX_SPAWN_PROXY_STACKSIZE 1024
#define CONFIG_TASK_SPAWN_DEFAULT_STACKSIZE 2048
#define CONFIG_LIBC_TMPDIR "/tmp"
#define CONFIG_LIBC_MAX_TMPFILE 32
#define CONFIG_ARCH_LOWPUTC 1
#define CONFIG_LIB_SENDFILE_BUFSIZE 512
#define CONFIG_ARCH_HAVE_TLS 1
#define CONFIG_HAVE_CXX 1
#define CONFIG_HAVE_CXXINITIALIZE 1
#define CONFIG_BUILTIN_PROXY_STACKSIZE 1024
#define CONFIG_EXAMPLES_HELLO 1
#define CONFIG_EXAMPLES_HELLO_PRIORITY 100
#define CONFIG_EXAMPLES_HELLO_STACKSIZE 2048
#define CONFIG_MYAPPS_GSENSOR 1
#define CONFIG_MYAPPS_GSENSOR_PRIORITY 100
#define CONFIG_MYAPPS_GSENSOR_STACKSIZE 2048
#define CONFIG_MYAPPS_GSENSOR_DEVPATH "/dev/gsensor"
#define CONFIG_MYAPPS_GSENSOR_GROUPSIZE 4
#define CONFIG_MYAPPS_GSENSOR_SWTRIG 1
#define CONFIG_MYAPPS_CONSOLE 1
#define CONFIG_MYAPPS_CONSOLE_STACKSIZE 2048
#define CONFIG_MYAPPS_CONSOLE_PRIORITY 100
#define CONFIG_MYAPPS_WAP 1
#define CONFIG_MYAPPS_WAP_PRIORITY 100
#define CONFIG_MYAPPS_WAP_STACKSIZE 2048
#define CONFIG_MYAPPS_WAP2 1
#define CONFIG_MYAPPS_WAP2_PRIORITY 100
#define CONFIG_MYAPPS_WAP2_STACKSIZE 2048
#define CONFIG_MYAPPS_START 1
#define CONFIG_MYAPPS_START_PRIORITY 100
#define CONFIG_MYAPPS_START_STACKSIZE 4096
#define CONFIG_MYAPPS_CONNECT 1
#define CONFIG_MYAPPS_CONNECT_PRIORITY 100
#define CONFIG_MYAPPS_CONNECT_STACKSIZE 2048
#define CONFIG_MYAPPS_START2 1
#define CONFIG_MYAPPS_START2_PRIORITY 100
#define CONFIG_MYAPPS_START2_STACKSIZE 16384
#define CONFIG_MYAPPS_CURTAIN 1
#define CONFIG_MYAPPS_CURTAIN_PRIORITY 100
#define CONFIG_MYAPPS_CURTAIN_STACKSIZE 2048
#define CONFIG_MYAPPS_FAN 1
#define CONFIG_MYAPPS_FAN_PRIORITY 100
#define CONFIG_MYAPPS_FAN_STACKSIZE 2048
#define CONFIG_MYAPPS_PUMP 1
#define CONFIG_MYAPPS_PUMP_PRIORITY 100
#define CONFIG_MYAPPS_PUMP_STACKSIZE 2048
#define CONFIG_NSH_LIBRARY 1
#define CONFIG_NSH_READLINE 1
#define CONFIG_NSH_LINELEN 64
#define CONFIG_NSH_MAXARGUMENTS 6
#define CONFIG_NSH_NESTDEPTH 3
#define CONFIG_NSH_BUILTIN_APPS 1
#define CONFIG_NSH_DISABLE_LOSMART 1
#define CONFIG_NSH_CODECS_BUFSIZE 128
#define CONFIG_NSH_PROC_MOUNTPOINT "/proc"
#define CONFIG_NSH_FILEIOSIZE 512
#define CONFIG_NSH_ROMFSETC 1
#define CONFIG_NSH_ROMFSMOUNTPT "/etc"
#define CONFIG_NSH_INITSCRIPT "init.d/rcS"
#define CONFIG_NSH_ROMFSDEVNO 0
#define CONFIG_NSH_ROMFSSECTSIZE 64
#define CONFIG_NSH_ARCHROMFS 1
#define CONFIG_NSH_FATDEVNO 1
#define CONFIG_NSH_FATSECTSIZE 512
#define CONFIG_NSH_FATNSECTORS 1024
#define CONFIG_NSH_FATMOUNTPT "/tmp"
#define CONFIG_NSH_CONSOLE 1
#define CONFIG_NSH_ARCHINIT 1
#define CONFIG_READLINE_HAVE_EXTMATCH 1
#define CONFIG_SYSTEM_READLINE 1
#define CONFIG_READLINE_ECHO 1

/* Sanity Checks *****************************************/

/* If this is an NXFLAT, external build, then make sure that
 * NXFLAT support is enabled in the base code.
 */

#if defined(__NXFLAT__) && !defined(CONFIG_NXFLAT)
# error "NXFLAT support not enabled in this configuration"
#endif

/* NXFLAT requires PIC support in the TCBs. */

#if defined(CONFIG_NXFLAT)
# undef CONFIG_PIC
# define CONFIG_PIC 1
#endif

/* Binary format support is disabled if no binary formats are
 * configured (at present, NXFLAT is the only supported binary.
 * format).
 */

#if !defined(CONFIG_NXFLAT) && !defined(CONFIG_ELF) && !defined(CONFIG_BUILTIN)
# undef CONFIG_BINFMT_DISABLE
# define CONFIG_BINFMT_DISABLE 1
#endif

/* The correct way to disable RR scheduling is to set the
 * timeslice to zero.
 */

#ifndef CONFIG_RR_INTERVAL
# define CONFIG_RR_INTERVAL 0
#endif

/* The correct way to disable filesystem supuport is to set the number of
 * file descriptors to zero.
 */

#ifndef CONFIG_NFILE_DESCRIPTORS
# define CONFIG_NFILE_DESCRIPTORS 0
#endif

/* If a console is selected, then make sure that there are resources for
 * three file descriptors and, if any streams are selected, also for three
 * file streams.
 *
 * CONFIG_DEV_CONSOLE means that a builtin console device exists at /dev/console
 * and can be opened during boot-up.  Other consoles, such as USB consoles, may
 * not exist at boot-upand have to be handled in a different way.  Three file
 * descriptors and three file streams are still needed.
 */

#if defined(CONFIG_DEV_CONSOLE) || defined(CONFIG_CDCACM_CONSOLE) || \
    defined(CONFIG_PL2303_CONSOLE)
# if CONFIG_NFILE_DESCRIPTORS < 3
#   undef CONFIG_NFILE_DESCRIPTORS
#   define CONFIG_NFILE_DESCRIPTORS 3
# endif

# if CONFIG_NFILE_STREAMS > 0 && CONFIG_NFILE_STREAMS < 3
#  undef CONFIG_NFILE_STREAMS
#  define CONFIG_NFILE_STREAMS 3
# endif

/* If no console is selected, then disable all builtin console devices */

#else
#  undef CONFIG_DEV_LOWCONSOLE
#  undef CONFIG_RAMLOG_CONSOLE
#endif

/* If priority inheritance is disabled, then do not allocate any
 * associated resources.
 */

#if !defined(CONFIG_PRIORITY_INHERITANCE) || !defined(CONFIG_SEM_PREALLOCHOLDERS)
# undef CONFIG_SEM_PREALLOCHOLDERS
# define CONFIG_SEM_PREALLOCHOLDERS 0
#endif

#if !defined(CONFIG_PRIORITY_INHERITANCE) || !defined(CONFIG_SEM_NNESTPRIO)
# undef CONFIG_SEM_NNESTPRIO
# define CONFIG_SEM_NNESTPRIO 0
#endif

/* If no file descriptors are configured, then make certain no
 * streams are configured either.
 */

#if CONFIG_NFILE_DESCRIPTORS == 0
# undef CONFIG_NFILE_STREAMS
# define CONFIG_NFILE_STREAMS 0
#endif

/* There must be at least one memory region. */

#ifndef CONFIG_MM_REGIONS
# define CONFIG_MM_REGIONS 1
#endif

/* If the end of RAM is not specified then it is assumed to be the beginning
 * of RAM plus the RAM size.
 */

#ifndef CONFIG_RAM_END
# define CONFIG_RAM_END (CONFIG_RAM_START+CONFIG_RAM_SIZE)
#endif

#ifndef CONFIG_RAM_VEND
# define CONFIG_RAM_VEND (CONFIG_RAM_VSTART+CONFIG_RAM_SIZE)
#endif

/* If the end of FLASH is not specified then it is assumed to be the beginning
 * of FLASH plus the FLASH size.
 */

#ifndef CONFIG_FLASH_END
# define CONFIG_FLASH_END (CONFIG_FLASH_START+CONFIG_FLASH_SIZE)
#endif

/* If no file streams are configured, then make certain that buffered I/O
 * support is disabled
 */

#if CONFIG_NFILE_STREAMS == 0
# undef CONFIG_STDIO_BUFFER_SIZE
# define CONFIG_STDIO_BUFFER_SIZE 0
#endif

/* If no standard C buffered I/O is not supported, then line-oriented buffering
 * cannot be supported.
 */

#if CONFIG_STDIO_BUFFER_SIZE == 0
# undef CONFIG_STDIO_LINEBUFFER
#endif

/* If the maximum message size is zero, then we assume that message queues
 * support should be disabled
 */

#if !defined(CONFIG_MQ_MAXMSGSIZE) || defined(CONFIG_DISABLE_MQUEUE)
# undef CONFIG_MQ_MAXMSGSIZE
# define CONFIG_MQ_MAXMSGSIZE 0
#endif

#if CONFIG_MQ_MAXMSGSIZE <= 0 && !defined(CONFIG_DISABLE_MQUEUE)
# define CONFIG_DISABLE_MQUEUE 1
#endif

/* If mountpoint support in not included, then no filesystem can be supported */

#ifdef CONFIG_DISABLE_MOUNTPOINT
# undef CONFIG_FS_FAT
# undef CONFIG_FS_ROMFS
# undef CONFIG_FS_NXFFS
# undef CONFIG_FS_SMARTFS
# undef CONFIG_FS_BINFS
# undef CONFIG_NFS
#endif

/* Check if any readable and writable filesystem (OR USB storage) is supported */

#if defined(CONFIG_FS_FAT) || defined(CONFIG_FS_ROMFS) || defined(CONFIG_USBMSC) || \
    defined(CONFIG_FS_NXFFS) || defined(CONFIG_FS_SMARTFS) || defined(CONFIG_FS_BINFS) || \
    defined(CONFIG_NFS) || defined(CONFIG_FS_PROCFS)
# undef  CONFIG_FS_READABLE
# define CONFIG_FS_READABLE 1
#endif

#if defined(CONFIG_FS_FAT) || defined(CONFIG_USBMSC) || defined(CONFIG_FS_NXFFS) || \
    defined(CONFIG_FS_SMARTFS) || defined(CONFIG_NFS)
# undef  CONFIG_FS_WRITABLE
# define CONFIG_FS_WRITABLE 1
#endif

/* There can be no network support with no socket descriptors */

#if CONFIG_NSOCKET_DESCRIPTORS <= 0
# undef CONFIG_NET
#endif

/* Conversely, if there is no network support, there is no need for
 * socket descriptors
 */

#ifndef CONFIG_NET
# undef CONFIG_NSOCKET_DESCRIPTORS
# define CONFIG_NSOCKET_DESCRIPTORS 0
#endif

/* Protocol support can only be provided on top of basic network support */

#ifndef CONFIG_NET
# undef CONFIG_NET_TCP
# undef CONFIG_NET_UDP
# undef CONFIG_NET_ICMP
#endif

/* NFS client can only be provided on top of UDP network support */

#if !defined(CONFIG_NET) || !defined(CONFIG_NET_UDP)
# undef CONFIG_NFS
#endif

/* Verbose debug and sub-system debug only make sense if debug is enabled */

#ifndef CONFIG_DEBUG
# undef CONFIG_DEBUG_VERBOSE
# undef CONFIG_DEBUG_SCHED
# undef CONFIG_DEBUG_MM
# undef CONFIG_DEBUG_PAGING
# undef CONFIG_DEBUG_DMA
# undef CONFIG_DEBUG_FS
# undef CONFIG_DEBUG_LIB
# undef CONFIG_DEBUG_BINFMT
# undef CONFIG_DEBUG_NET
# undef CONFIG_DEBUG_USB
# undef CONFIG_DEBUG_GRAPHICS
# undef CONFIG_DEBUG_GPIO
# undef CONFIG_DEBUG_SPI
# undef CONFIG_DEBUG_HEAP
#endif

/* User entry point. This is provided as a fall-back to keep compatibility
 * with existing code, for builds which do not define CONFIG_USER_ENTRYPOINT.
 */

#ifndef CONFIG_USER_ENTRYPOINT
# define CONFIG_USER_ENTRYPOINT main
#endif

#endif /* __INCLUDE_NUTTX_CONFIG_H */
