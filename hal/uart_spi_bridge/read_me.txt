/******************************************************************************
 *                                Overview
 ******************************************************************************/
 * UART-SPI bridge application

 * This application implements the SPI Master and act as UART-SPI Bridge.
 * The uart_spi_bridge sample application demonstrates the signaling protocol
 * to be implemented on the customer MCU to communicate over SPI using the
 * WICED_HCI messages.
 * The bridge application implements the following functionalities
 *   1. Receives WICED_HCI Commands from MCU/Host over PUART and
 *      forwards the commands over SPI to the Slave device.
 *   2. Receives WICED_HCI Events from the connected WICED SPI device
 *      and forwards them over PUART to the connected host.
 *   3. Intercepts the HCI_CONTROL_AUDIO_DATA request from the WICED
 *      SPI device and responds with a stored sine wave to the WICED device.

/******************************************************************************
 *                                Instructions
 ******************************************************************************/
 * * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED Bluetooth ( 20706 ) evaluation board into your computer
 * 2. Build and download the uart_spi_bridge application to a 20706
 *    board(SPI Master).
 * 3. Build and download the hci_av_source application to another 20706
 *    board(SPI Slave).
 * 4. Connect the SPI signals between master and slave.
 *    -------------------------------------------------
 *    |             Master        |        Slave       |
 *    -------------------------------------------------
 *    | SPI CLK    P36(J19.6)     |     P3(J22.8)      |
 *    -------------------------------------------------
 *    | SPI MOSI   P0(J22.6)      |     P0(J22.6)      |
 *    -------------------------------------------------
 *    | SPI MISO   P25(J19.4)     |     P25(J19.4)     |
 *    -------------------------------------------------
 *    | SPI CS     P26(J22.4)     |     P2(J22.5)      |
 *    -------------------------------------------------
 *    | SLAVE READY P12(J22.7)    |     P15(J22.3)     |
 *    -------------------------------------------------
 *    | GND         J19.8         |     J19.8          |
 *    -------------------------------------------------
 * 5. Open the Client Control application. Select  the PUART port on the 20706
 *    running the uart_spi_bridge application at 115200 baud rate
 *    without flow control
 * 6. Reset the 20706 that runs the uart_spi_bridge application.
 * 7. Reset the 20706 that runs the watch application.
 * 6. Use ClientControl application to send various commands
 */
