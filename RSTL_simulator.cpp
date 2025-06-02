#include <iostream>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <ctype.h>

#define RSTL_HARDWARE_SPEED			B150

static int configureSerialPort(const char *DeviceName);

int SerialDevice;


int main() {
	SerialDevice = configureSerialPort( "/dev/ttyS4" );
	if (SerialDevice < 0){
		std::cout << "Port not opened" << std::endl;
		return 0;
	}
    std::cout << "Port open and configured" << std::endl;

    struct termios oldSettings, newSettings;
    tcgetattr(STDIN_FILENO, &oldSettings);
    newSettings = oldSettings;
    newSettings.c_lflag &= ~(ICANON | ECHO); // Tryb niekanoniczny, bez echa
    tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);

    while (true) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        struct timeval timeout = {0, 0}; // wait 0 seconds
        int ready = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
        if (ready > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
            char KeyCode;
            if (read(STDIN_FILENO, &KeyCode, 1) == 1) {
        		if ((27 == KeyCode) || ('q' == KeyCode) || (1 == KeyCode)){  // Esc, q, Ctrl+A
                    break;
                }
                std::cout << "Pressed: " << (int)KeyCode << "\n";
            }
        }










        usleep(100000);
    }

	close( SerialDevice );
    std::cout << "Port closed" << std::endl;

    tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
    return 0;
}


// This function opens and configures a serial port
static int configureSerialPort(const char *DeviceName){
    int FileHandler;
    struct termios PortSettings;

    FileHandler = open(DeviceName, O_RDWR | O_NOCTTY | O_SYNC);
    if (FileHandler == -1) {
        return -1;
    }

    if (tcgetattr(FileHandler, &PortSettings) != 0) {
        close(FileHandler);
        return -1;
    }

    cfsetispeed(&PortSettings, RSTL_HARDWARE_SPEED);
    cfsetospeed(&PortSettings, RSTL_HARDWARE_SPEED);

    PortSettings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    PortSettings.c_oflag = 0;                            // Disable output processing
    PortSettings.c_lflag = 0;                            // Mode 'raw'

    PortSettings.c_cflag &= ~(CSIZE | CSTOPB | PARODD | CRTSCTS);
    PortSettings.c_cflag |= (CS8 | PARENB | CREAD | CLOCAL);

    PortSettings.c_cc[VMIN]  = 0;                        // Minimum number of bytes to read
    PortSettings.c_cc[VTIME] = 0;                        // Timeout in tenth of a second

    // apply the configuration
    if (tcsetattr(FileHandler, TCSANOW, &PortSettings) != 0) {
        close(FileHandler);
        return -1;
    }

    return FileHandler;
}


