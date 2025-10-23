import serial
import threading
import time

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200


class RobotHardwareInterface:
# vai ser responsável pela comunicação com a ESP. Pega detalhes do protocolo sereal p resto do ros

	def __init__(self):
		self.connection = None
		try:
			self.connection = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
			time.sleep(2)
			print("Interface de Hardware conectada ao ESP32.")
		except serial.SerialException as e:
			print(f"Erro Crítico: Nao foi possível conectar ao hardware. {e}")
			return

		self.current_left_wheel_velocity = 0.0
		self.current_right_wheel_velocity = 0.0

		self.is_running = True
		self.receiver_thread = threading.Thread(target=self._read_data_loop)
		self.receiver_thread.daemon = True
		self.receiver_thread.start()


	def _read_data_loop(self):
	# loop privado q roda em thread p ler dados da ESP

		while self.is_running:
			if self.connection and self.connection.in_waiting > 0:
				try:

					# ESP envia no formato vel_esq;vel_dir\n (?)
					decoded_line = self.connection.readline().decode('utf-8').strip()
					
					if decoded_line:  
						parts = decoded_line.split(';')

						if len(parts) == 2:
							# Atualizar variáveis de estado
							self.current_left_wheel_velocity = float(parts[0])
							self.current_right_wheel_velocity = float(parts[1])
							print(f"Dados ESP: L={parts[0]}, R={parts[1]}")
						else:
							print(f"Formato inválido: {decoded_line}")
                            
				except (UnicodeDecodeError, ValueError) as e:
					print(f"Dado malformado: {e}")
					pass
			time.sleep(0.01)#pequeno delay


	def send_velocity_command(self, left_velocity, right_velocity):
	# p enviar comando d veloc p rodas
	# precisa formatar comando no formato esperado pela ESP

		command = f"{left_velocity:.3f};{right_velocity:.3f}\n"
        
		if self.connection:
			try:
				self.connection.write(command.encode('utf-8'))
				print(f"Comando enviado: {command.strip()}")
			except serial.SerialException as e:
				print(f"Erro ao enviar comando: {e}")


	def get_current_state(self):
	# p retornar estado mais recente lido do hardw

		print(f"Retornou o estado atual")
		return (self.current_left_wheel_velocity, self.current_right_wheel_velocity)


	def close(self):
	# terminar comunicacao

		self.is_running = False
		self.receiver_thread.join(timeout=1)
		if self.connection:
			self.connection.close()
		print("Interface de Hardware desconectada.")
