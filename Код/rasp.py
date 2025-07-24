import serial
import sys
import tty
import termios

# Настройка UART (замените порт при необходимости)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def send_command(cmd):
    ser.write(cmd.encode())
    print(f"Отправлено: {cmd}")

def get_key():
    # Настройка терминала для чтения клавиш без Enter
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

if __name__ == "__main__":
    print("Управление: W/A/S/D/X/C. Выход: Q")
    try:
        while True:
            key = get_key().upper()
            if key in ['W', 'A', 'S', 'D', 'X', 'C']:
                send_command(key)
            elif key == 'Q':
                print("Выход...")
                break

    except KeyboardInterrupt:
        print("\nПрограмма завершена.")
    finally:
        ser.close()
