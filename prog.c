#define _CRT_SECURE_NO_WARNINGS

#define BUFFER_SIZE 10

#define N_size 10
#include <windows.h>
#include <stdio.h>
#include <tchar.h>
#include <stdlib.h>

void main(void) {

	DCB dcb;
	HANDLE com_port;
	BOOL fSuccess = 1;
	FILE* fp;

	char* cp_name = L"COM1";
	char name[] = "uart.txt";
	char buffer[BUFFER_SIZE] = { 0 };

	DWORD dwRead;

	int i = 0, j = 0;
	int a[N_size] = { 0 };

	/*Открытие и настройка COM порта*/

	com_port = CreateFile(cp_name,
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		0,
		0
	);

	if (com_port == INVALID_HANDLE_VALUE) {
		printf("CreateFile failed with error %d.\n", GetLastError());
		return (1);
	}

	fSuccess = GetCommState(com_port, &dcb);
	if (!fSuccess) {
		printf("GetCommState failed with error %d.\n", GetLastError());
		return (2);
	}

	dcb.BaudRate = CBR_14400;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;

	fSuccess = SetCommState(com_port, &dcb);

	if (!fSuccess) {
		printf("SetCommState failed with error %d.\n", GetLastError());
		return (3);
	}

	printf("Serial port configured.\n");
	/*Настройка записи в файл*/

	if ((fp = fopen(&name, "w")) == NULL) {
		printf("File hasn`t been opened");
		return(4);

	}

	/*Запись данных из порта в файл*/

	while (1) {
		if (ReadFile(com_port, buffer, BUFFER_SIZE, &dwRead, NULL)) {
			if (buffer[0] != NULL) {
				if (i < N_size) {
					printf("%s\n", buffer);
					a[i] = atoi(buffer);
					memset(buffer, 0, BUFFER_SIZE);
					i++;
				}

				else {
					break;

				}

			}

		}

		else {
			printf("ReadFile failed with error %d.\n", GetLastError());
			break;

		}

	}

	for (j = 0; j < N_size; j++) {
		printf("%d\n", a[j]);
		fprintf(fp, "%d\n", a[j]);

	}

	CloseHandle(com_port);
	fclose(fp);;
}