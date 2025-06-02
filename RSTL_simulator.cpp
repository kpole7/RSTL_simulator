#include <iostream>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define RSTL_HARDWARE_SPEED			B150

static int configureSerialPort(const char *DeviceName);

int SerialDevice;





void cleanup() {
    std::cout << "\nZamykanie programu...\n";
}

int main() {
    struct termios oldSettings, newSettings;
    tcgetattr(STDIN_FILENO, &oldSettings); // Zapisz ustawienia terminala
    newSettings = oldSettings;
    newSettings.c_lflag &= ~(ICANON | ECHO); // Tryb niekanoniczny + bez echa
    tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);

    char ch;
    std::cout << "Naciśnij Ctrl+A lub 'q' aby wyjść...\n";
    while (read(STDIN_FILENO, &ch, 1) == 1 && ch != 'q') {
        if (ch == 1) { // Ctrl+A to kod ASCII 1
            cleanup();
            break;
        }
        std::cout << "Wciśnięto: " << (int)ch << "\n";
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings); // Przywróć ustawienia
    return 0;
}





#if 0

int main(int argc, char **argv) {
	std::cout << "Hello World" << std::endl;

	SerialDevice = configureSerialPort( "ttyS4" );
	if (SerialDevice < 0){
		std::cout << "Port not opened" << std::endl;
		return 0;
	}


	while(1){



	}

	close( SerialDevice );
	return 0;
}
#endif


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


