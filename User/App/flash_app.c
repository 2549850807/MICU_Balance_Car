#include "flash_app.h"

uint8_t flash_write_buffer[SPI_FLASH_PAGE_SIZE];
uint8_t flash_read_buffer[SPI_FLASH_PAGE_SIZE];

unsigned char start_count = 0;

void Flash_Init(void)
{
  Uart_Printf(DEBUG_UART, "Flash_Init ......\r\n");
  Flash_SPI_Test();
  
  spi_flash_buffer_read(&start_count, 0x1000, 1);
  start_count = start_count + 1;
  
  spi_flash_sector_erase(0x1000);
  spi_flash_buffer_write(&start_count, 0x1000, 1);
  
  Uart_Printf(DEBUG_UART, "start_count:%d\r\n", (int)start_count);
}

void Flash_Task(void)
{
  
}

void Flash_SPI_Test(void) 
{
    uint32_t flash_id;
    uint8_t write_buffer[SPI_FLASH_PAGE_SIZE];
    uint8_t read_buffer[SPI_FLASH_PAGE_SIZE];
    uint32_t test_addr = 0x000000; // ���Ե�ַ��ѡ��һ����������ʼ

    Uart_Printf(DEBUG_UART, "SPI FLASH Test Start\r\n");

    // 1. ��ʼ��SPI Flash���� (��Ҫ��CS����״̬)
    spi_flash_init();
    Uart_Printf(DEBUG_UART, "SPI Flash Initialized.\r\n");

    // 2. ��ȡFlash ID
    flash_id = spi_flash_read_id();
    Uart_Printf(DEBUG_UART, "Flash ID: 0x%lX\r\n", flash_id);
    // ����Ը������оƬ�ֲ���ID�Ƿ���ȷ������ GD25Q64��ID������ 0xC84017

    // 3. ����һ������ (��Сͨ��Ϊ4KB)
    // ע�⣺����������ʱ�ϳ�
    Uart_Printf(DEBUG_UART, "Erasing sector at address 0x%lX...\r\n", test_addr);
    spi_flash_sector_erase(test_addr);
    Uart_Printf(DEBUG_UART, "Sector erased.\r\n");

    // (��ѡ) У���������ȡһҳ���ݣ�����Ƿ�ȫΪ0xFF
    spi_flash_buffer_read(read_buffer, test_addr, SPI_FLASH_PAGE_SIZE);
    int erased_check_ok = 1;
    for (int i = 0; i < SPI_FLASH_PAGE_SIZE; i++) {
        if (read_buffer[i] != 0xFF) {
            erased_check_ok = 0;
            break;
        }
    }
    if (erased_check_ok) {
        Uart_Printf(DEBUG_UART, "Erase check PASSED. Sector is all 0xFF.\r\n");
    } else {
        Uart_Printf(DEBUG_UART, "Erase check FAILED.\r\n");
    }

    // 4. ׼��д������ (д��һҳ)
    const char* message = "Hello from STM32 to SPI FLASH! Microunion Studio Test - 12345.";
    uint16_t data_len = strlen(message);
    if (data_len >= SPI_FLASH_PAGE_SIZE) {
        data_len = SPI_FLASH_PAGE_SIZE -1; // ȷ��������ҳ��С
    }
    memset(write_buffer, 0, SPI_FLASH_PAGE_SIZE);
    memcpy(write_buffer, message, data_len);
    write_buffer[data_len] = '\0'; // ȷ���ַ�������

    Uart_Printf(DEBUG_UART, "Writing data to address 0x%lX: \"%s\"\r\n", test_addr, write_buffer);
    // ʹ�� spi_flash_buffer_write (���Դ����ҳ��������ֻдһҳ��)
    // ����ֱ���� spi_flash_page_write ���ȷ����һҳ��
    spi_flash_buffer_write(write_buffer, test_addr, SPI_FLASH_PAGE_SIZE); // д����ҳ���ݣ���������0
    // spi_flash_page_write(write_buffer, test_addr, data_len + 1); // ֻд����Ч����
    Uart_Printf(DEBUG_UART, "Data written.\r\n");

    // 5. ��ȡд�������
    Uart_Printf(DEBUG_UART, "Reading data from address 0x%lX...\r\n", test_addr);
    memset(read_buffer, 0, SPI_FLASH_PAGE_SIZE);
    spi_flash_buffer_read(read_buffer, test_addr, SPI_FLASH_PAGE_SIZE);
    Uart_Printf(DEBUG_UART, "Data read: \"%s\"\r\n", read_buffer);

    // 6. У������
    if (memcmp(write_buffer, read_buffer, SPI_FLASH_PAGE_SIZE) == 0) {
        Uart_Printf(DEBUG_UART, "Data VERIFIED! Write and Read successful.\r\n");
    } else {
        Uart_Printf(DEBUG_UART, "Data VERIFICATION FAILED!\r\n");
    }

    Uart_Printf(DEBUG_UART, "SPI FLASH Test End\r\n");
}
