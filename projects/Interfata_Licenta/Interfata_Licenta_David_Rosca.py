import pygame
import numpy as np
import serial
import threading
import time
import re
from collections import deque
import sys
import math

pygame.init()

class SeismicMonitor:
    def __init__(self):
        # Setări pentru afișaj
        self.WINDOW_WIDTH = 1200
        self.WINDOW_HEIGHT = 800
        self.fullscreen_mode = False
        self.original_size = (1200, 800)
        self.screen = pygame.display.set_mode((self.WINDOW_WIDTH, self.WINDOW_HEIGHT))
        pygame.display.set_caption("MONITORIZAREA ACTIVITATII SEISMICE")
        
        # Culori - culori mai întunecate
        self.BACKGROUND_COLOR = (173, 216, 230)
        self.WHITE = (255, 255, 255)
        self.NAVY = (0, 0, 128)
        self.RED = (200, 0, 0)
        self.GREEN = (0, 100, 0)
        self.BLUE = (0, 0, 200)
        self.GRAY = (80, 80, 80)
        self.DARK_GREEN = (0, 80, 0)
        self.BRIGHT_RED = (220, 20, 20)
        self.DARK_RED = (120, 0, 0)
        self.BRIGHT_ORANGE = (200, 140, 0)
        self.BRIGHT_YELLOW = (200, 200, 0)
        self.BRIGHT_GREEN = (0, 180, 0)
        
        # Culori pentru cadrane actualizate
        self.LIGHT_GREEN = (144, 238, 144)      # Verde deschis pentru caseta accelerației
        self.ORANGE = (255, 140, 0)             # Portocaliu pentru caseta frecvenței
        self.RED_PURPLE = (220, 20, 60)         # Roșu-purpuriu pentru caseta scalei Richter
        self.LIGHT_YELLOW = (200, 200, 150)     # Durata
        self.LIGHT_PINK = (255, 192, 203)       # Roz deschis pentru fereastra duratei
        self.CREAM = (255, 248, 220)            # Culoare cremă pentru caseta ADC
        
        # Fonturi
        self.title_font = pygame.font.Font(None, 48)
        self.box_font = pygame.font.Font(None, 28)  # Mărit de la 24 la 28
        self.box_font.set_bold(True)
        self.value_font = pygame.font.Font(None, 36)  # Mărit de la 32 la 36
        self.value_font.set_bold(True)
        self.small_font = pygame.font.Font(None, 20)
        self.medium_font = pygame.font.Font(None, 22)
        self.terminal_font = pygame.font.Font(None, 14)
        
        # Conexiune serială
        self.serial_port = None
        self.running = False
        self.connected = False
        self.serial_port_input = "/dev/ttyUSB1"
        self.input_active = False
        
        # Stocare date ADC - 6 canale pentru fiecare ADC (ADC0 și ADC1)
        self.adc0_channels = {i: deque(maxlen=2000) for i in range(6)}  # ADC0 canale 0-5
        self.adc1_channels = {i: deque(maxlen=2000) for i in range(6)}  # ADC1 canale 0-5
        
        # Conversie simplă: 0 ADC = 0g, 4095 ADC = 8g
        self.adc_to_g_factor = 8.0 / 4095.0  # Factor de conversie
        
        # Accelerații pentru fiecare canal (interval 0-8g) - 6 canale finale
        self.channel_accelerations = {i: 1.0 for i in range(6)}
        
        # Stabilitatea detectării cutremurului - analizează multiple mostre consecutive
        self.earthquake_history_length = 10  # Analizează ultimele 10 mostre pentru stabilitate
        self.earthquake_consecutive_threshold = 7  # Necesită 7 din 10 mostre pentru confirmare
        
        # Accelerații medii per axă 
        self.axis_accelerations = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        
        # Suma celor trei medii ale axelor
        self.axis_sum = 0.0
        
        # Media combinată pentru calculul frecvenței FFT
        self.combined_average = 0.0
        
        # Date pentru grafice
        self.acceleration_data = {
            'X': deque(maxlen=2000),
            'Y': deque(maxlen=2000), 
            'Z': deque(maxlen=2000)
        }
        
        # Scara Richter bazată pe mostre ADC în timp real
        self.magnitude_data = deque(maxlen=800)
        self.current_magnitude = 0.1  # CORECTAT: Început cu 0.1 în loc de 0.0
        self.earthquake_level = "CUTREMUR INEXISTENT"
        self.earthquake_detected = False
        
        # Urmărirea timpului de detectare a cutremurului 
        self.earthquake_detection_time = None
        self.last_earthquake_time = None
        
        # Analiza frecvenței - generarea undelor osciloscop 
        self.frequency_data = deque(maxlen=500)
        self.current_frequency = 0.0
        self.oscilloscope_wave_data = deque(maxlen=1000)  # Pentru forma de undă a osciloscopului
        self.wave_time = 0.0  # Pentru generarea continuă de unde
        
        # Calculul vitezei - integrala accelerațiilor
        self.current_velocity = 0.0
        self.velocity_integral = 0.0
        
        # Linii terminal
        self.terminal_lines = deque(maxlen=30)
        
        # Starea inițializării sistemului
        self.spi_engine_initialized = False
        self.axi_pwm_custom_initialized = False
        self.clk_generator_initialized = False
        
        # Eșantionare și cronometrare - frecvență realistă seismică
        self.sample_count = 0
        self.start_time = time.time()
        self.sample_rate = 200.0  # Hz - frecvență standard pentru seismografie (50-200 Hz)
        self.dt = 1.0 / self.sample_rate
        
        # Stări ferestre
        self.windows = {
            'acceleration': False,
            'magnitude': False,
            'frequency': False,
            'duration': False,
            'adc': False
        }
        
        # Animația undelor seismice
        self.wave_animation_time = 0.0
        self.left_wave_points = []
        self.right_wave_points = []
        
        self.setup_ui()
        self.clock = pygame.time.Clock()
        
    def setup_ui(self):
        center_x = self.WINDOW_WIDTH // 2
        center_y = self.WINDOW_HEIGHT // 2
        box_w, box_h, spacing = 250, 140, 30
        
        top_y = center_y - box_h - spacing//2 - 40
        bottom_y = center_y + spacing//2 - 40
        adc_y = bottom_y + box_h + spacing
        left_x = center_x - box_w - spacing//2
        right_x = center_x + spacing//2
        
        self.boxes = {
            'acceleration': pygame.Rect(left_x, top_y, box_w, box_h),
            'richter': pygame.Rect(right_x, top_y, box_w, box_h),
            'frequency': pygame.Rect(left_x, bottom_y, box_w, box_h),
            'duration': pygame.Rect(right_x, bottom_y, box_w, box_h),
            'adc': pygame.Rect(center_x - (2*box_w + spacing)//2, adc_y, 2*box_w + spacing, box_h)
        }
        
        self.port_input_rect = pygame.Rect(center_x - 150, 100, 200, 30)
        self.connect_button_rect = pygame.Rect(center_x + 70, 100, 100, 30)
        
        # Buton ecran complet
        self.fullscreen_button_rect = pygame.Rect(self.WINDOW_WIDTH - 120, 20, 100, 35)
    
    def toggle_fullscreen(self):
        """Comută între modul ecran complet și fereastră"""
        try:
            if self.fullscreen_mode:
                self.WINDOW_WIDTH, self.WINDOW_HEIGHT = self.original_size
                self.screen = pygame.display.set_mode((self.WINDOW_WIDTH, self.WINDOW_HEIGHT))
                self.fullscreen_mode = False
            else:
                info = pygame.display.Info()
                self.WINDOW_WIDTH = info.current_w
                self.WINDOW_HEIGHT = info.current_h
                self.screen = pygame.display.set_mode((self.WINDOW_WIDTH, self.WINDOW_HEIGHT), pygame.FULLSCREEN)
                self.fullscreen_mode = True
            
            self.setup_ui()
            
        except Exception as e:
            print(f"Eroare comutare ecran complet: {e}")
            self.WINDOW_WIDTH, self.WINDOW_HEIGHT = self.original_size
            self.screen = pygame.display.set_mode((self.WINDOW_WIDTH, self.WINDOW_HEIGHT))
            self.fullscreen_mode = False
            self.setup_ui()
    
    def draw_fullscreen_button(self):
        """Desenează butonul de comutare ecran complet"""
        button_color = self.DARK_GREEN if not self.fullscreen_mode else self.BRIGHT_ORANGE
        pygame.draw.rect(self.screen, button_color, self.fullscreen_button_rect)
        pygame.draw.rect(self.screen, self.NAVY, self.fullscreen_button_rect, 2)
        
        button_font = pygame.font.Font(None, 18)
        button_font.set_bold(True)
        button_text = "Fullscreen" if not self.fullscreen_mode else "Windowed"
        text_surface = button_font.render(button_text, True, self.WHITE)
        text_rect = text_surface.get_rect(center=self.fullscreen_button_rect.center)
        self.screen.blit(text_surface, text_rect)
        
        mouse_pos = pygame.mouse.get_pos()
        if self.fullscreen_button_rect.collidepoint(mouse_pos):
            pygame.draw.rect(self.screen, self.WHITE, self.fullscreen_button_rect, 1)

    def draw_beautiful_title(self):
        """Desenează un titlu frumos cu efecte de gradient și umbră"""
        title_text = "MONITORIZAREA ACTIVITĂȚII SEISMICE"
        
        # Creează fontul principal al titlului - mai mare și mai elegant
        main_font = pygame.font.Font(None, 56)
        
        # Efect de umbră - desenează multiple versiuni deplasate pentru adâncime
        shadow_offsets = [(3, 3), (2, 2), (1, 1)]
        shadow_colors = [(100, 100, 100), (120, 120, 120), (140, 140, 140)]
        
        # Calculează poziția centrală
        center_x = self.WINDOW_WIDTH // 2
        title_y = 40
        
        # Desenează straturile de umbră
        for i, (offset_x, offset_y) in enumerate(shadow_offsets):
            shadow_surface = main_font.render(title_text, True, shadow_colors[i])
            shadow_rect = shadow_surface.get_rect(center=(center_x + offset_x, title_y + offset_y))
            self.screen.blit(shadow_surface, shadow_rect)
        
        # Titlul principal cu culoare elegantă (albastru închis)
        title_color = (25, 25, 112)  # Albastru de miezul nopții
        title_surface = main_font.render(title_text, True, title_color)
        title_rect = title_surface.get_rect(center=(center_x, title_y))
        self.screen.blit(title_surface, title_rect)
        
        # Adaugă linii decorative pe ambele părți
        line_y = title_y + 35
        line_length = 150
        line_thickness = 3
        
        # Linia decorativă din stânga
        left_line_start = (center_x - title_rect.width//2 - line_length - 20, line_y)
        left_line_end = (center_x - title_rect.width//2 - 20, line_y)
        pygame.draw.line(self.screen, title_color, left_line_start, left_line_end, line_thickness)
        
        # Linia decorativă din dreapta
        right_line_start = (center_x + title_rect.width//2 + 20, line_y)
        right_line_end = (center_x + title_rect.width//2 + line_length + 20, line_y)
        pygame.draw.line(self.screen, title_color, right_line_start, right_line_end, line_thickness)
        
        # Adaugă diamante decorative mici la capetele liniilor
        diamond_size = 8
        
        # Diamantul din stânga
        left_diamond_points = [
            (left_line_start[0] - diamond_size, line_y),
            (left_line_start[0], line_y - diamond_size//2),
            (left_line_start[0] + diamond_size, line_y),
            (left_line_start[0], line_y + diamond_size//2)
        ]
        pygame.draw.polygon(self.screen, title_color, left_diamond_points)
        
        # Diamantul din dreapta
        right_diamond_points = [
            (right_line_end[0] - diamond_size, line_y),
            (right_line_end[0], line_y - diamond_size//2),
            (right_line_end[0] + diamond_size, line_y),
            (right_line_end[0], line_y + diamond_size//2)
        ]
        pygame.draw.polygon(self.screen, title_color, right_diamond_points)
        
        # Adaugă subtitlul
        subtitle_font = pygame.font.Font(None, 28)
        subtitle_text = "SISTEM DE MONITORIZARE ÎN TIMP REAL"
        subtitle_surface = subtitle_font.render(subtitle_text, True, (70, 70, 70))
        subtitle_rect = subtitle_surface.get_rect(center=(center_x, title_y + 50))
        self.screen.blit(subtitle_surface, subtitle_rect)

    def draw_seismic_waves(self):
        """Desenează unde seismice animate pe părțile stânga și dreapta ale interfeței"""
        # Actualizează timpul animației
        self.wave_animation_time += 0.05
        
        # Determină proprietățile undei pe baza nivelului cutremurului
        if self.earthquake_level == "CUTREMUR DEVASTATOR":
            wave_amplitude = 40 + math.sin(self.wave_animation_time * 3) * 20
            wave_frequency = 0.8
            wave_color = (220, 20, 20)  # Roșu strălucitor
            wave_thickness = 6
        elif self.earthquake_level == "CUTREMUR PUTERNIC":
            wave_amplitude = 25 + math.sin(self.wave_animation_time * 2) * 15
            wave_frequency = 0.6
            wave_color = (255, 100, 0)  # Portocaliu
            wave_thickness = 5
        elif self.earthquake_level == "CUTREMUR MODERAT":
            wave_amplitude = 15 + math.sin(self.wave_animation_time * 1.5) * 10
            wave_frequency = 0.4
            wave_color = (255, 200, 0)  # Galben
            wave_thickness = 4
        else:
            wave_amplitude = 8 + math.sin(self.wave_animation_time) * 5
            wave_frequency = 0.2
            wave_color = (0, 150, 100)  # Verde
            wave_thickness = 3
        
        # Intensitate adițională din magnitudinea curentă
        magnitude_multiplier = 1.0 + (self.current_magnitude / 9.0) * 2.0
        wave_amplitude *= magnitude_multiplier
        
        # Desenează undele din partea stângă
        left_waves = []
        wave_height = self.WINDOW_HEIGHT - 200  # Lasă spațiu pentru titlu și alerta de jos
        wave_start_y = 120  # Începe sub titlu
        
        for y in range(wave_start_y, wave_start_y + wave_height, 3):
            # Creează modelul undei
            progress = (y - wave_start_y) / wave_height
            wave_x = wave_amplitude * math.sin(progress * wave_frequency * 10 + self.wave_animation_time)
            
            # Adaugă multiple unde armonice pentru complexitate
            wave_x += (wave_amplitude * 0.3) * math.sin(progress * wave_frequency * 20 + self.wave_animation_time * 2)
            wave_x += (wave_amplitude * 0.2) * math.sin(progress * wave_frequency * 30 + self.wave_animation_time * 3)
            
            # Adaugă perturbații specifice cutremurului
            if self.earthquake_level in ["CUTREMUR DEVASTATOR", "CUTREMUR PUTERNIC"]:
                disturbance = (wave_amplitude * 0.5) * math.sin(progress * 50 + self.wave_animation_time * 8)
                wave_x += disturbance
            
            x_pos = int(20 + wave_x)
            x_pos = max(5, min(100, x_pos))  # Păstrează în limitele din stânga
            left_waves.append((x_pos, y))
        
        # Desenează undele din partea dreaptă (oglindite)
        right_waves = []
        for y in range(wave_start_y, wave_start_y + wave_height, 3):
            progress = (y - wave_start_y) / wave_height
            wave_x = wave_amplitude * math.sin(progress * wave_frequency * 10 + self.wave_animation_time + math.pi)
            
            # Adaugă armonice
            wave_x += (wave_amplitude * 0.3) * math.sin(progress * wave_frequency * 20 + self.wave_animation_time * 2 + math.pi)
            wave_x += (wave_amplitude * 0.2) * math.sin(progress * wave_frequency * 30 + self.wave_animation_time * 3 + math.pi)
            
            # Adaugă perturbații de cutremur
            if self.earthquake_level in ["CUTREMUR DEVASTATOR", "CUTREMUR PUTERNIC"]:
                disturbance = (wave_amplitude * 0.5) * math.sin(progress * 50 + self.wave_animation_time * 8 + math.pi)
                wave_x += disturbance
            
            x_pos = int(self.WINDOW_WIDTH - 20 - wave_x)
            x_pos = max(self.WINDOW_WIDTH - 100, min(self.WINDOW_WIDTH - 5, x_pos))  # Păstrează în limitele din dreapta
            right_waves.append((x_pos, y))
        
        # Desenează liniile undei cu efect de gradient
        if len(left_waves) > 1:
            # Creează efect de gradient pentru undele din stânga
            for i in range(len(left_waves) - 1):
                alpha = 1.0 - (i / len(left_waves))
                color_intensity = int(255 * alpha)
                gradient_color = (
                    min(255, int(wave_color[0] * alpha)),
                    min(255, int(wave_color[1] * alpha)),
                    min(255, int(wave_color[2] * alpha))
                )
                
                if i < len(left_waves) - 1:
                    pygame.draw.line(self.screen, gradient_color, left_waves[i], left_waves[i + 1], wave_thickness)
        
        if len(right_waves) > 1:
            # Creează efect de gradient pentru undele din dreapta
            for i in range(len(right_waves) - 1):
                alpha = 1.0 - (i / len(right_waves))
                gradient_color = (
                    min(255, int(wave_color[0] * alpha)),
                    min(255, int(wave_color[1] * alpha)),
                    min(255, int(wave_color[2] * alpha))
                )
                
                if i < len(right_waves) - 1:
                    pygame.draw.line(self.screen, gradient_color, right_waves[i], right_waves[i + 1], wave_thickness)
        
        # Adaugă particule/puncte de undă pentru efect suplimentar
        if self.earthquake_level in ["CUTREMUR DEVASTATOR", "CUTREMUR PUTERNIC"]:
            for i in range(0, len(left_waves), 15):
                if i < len(left_waves):
                    particle_size = int(3 + math.sin(self.wave_animation_time * 5 + i * 0.1) * 2)
                    pygame.draw.circle(self.screen, wave_color, left_waves[i], particle_size)
            
            for i in range(0, len(right_waves), 15):
                if i < len(right_waves):
                    particle_size = int(3 + math.sin(self.wave_animation_time * 5 + i * 0.1 + math.pi) * 2)
                    pygame.draw.circle(self.screen, wave_color, right_waves[i], particle_size)
        
        # Stochează punctele pentru potențială utilizare viitoare
        self.left_wave_points = left_waves
        self.right_wave_points = right_waves
        
    def start_serial_connection(self):
        try:
            self.serial_port = serial.Serial(self.serial_port_input, 115200, timeout=1)
            self.running = True
            self.connected = True
            self.start_time = time.time()
            self.sample_count = 0
            
            # Resetează starea inițializării
            self.spi_engine_initialized = False
            self.axi_pwm_custom_initialized = False
            self.clk_generator_initialized = False
            
            # Resetează timpii de detectare a cutremurului 
            self.earthquake_detection_time = None
            self.last_earthquake_time = None
            
            # Resetează toate colecțiile de date
            for i in range(6):
                self.adc0_channels[i].clear()
                self.adc1_channels[i].clear()
            for axis in ['X', 'Y', 'Z']:
                self.acceleration_data[axis].clear()
            
            self.velocity_integral = 0.0
            self.wave_time = 0.0
            
            # CORECTAT: Resetează magnitudinea la 0.1 în loc de 0.0
            self.current_magnitude = 0.1
            
            threading.Thread(target=self.read_serial_data, daemon=True).start()
            print(f"Conectat la {self.serial_port_input}")
        except Exception as e:
            print(f"Conectarea a eșuat: {e}")
            
    def stop_serial_connection(self):
        self.running = False
        self.connected = False
        
        # Resetează toate valorile când se deconectează
        for i in range(6):
            self.channel_accelerations[i] = 0.0
        self.axis_accelerations = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        self.axis_sum = 0.0
        self.combined_average = 0.0
        self.current_magnitude = 0.1  # CORECTAT: 0.1 în loc de 0.0
        self.current_frequency = 0.0
        self.current_velocity = 0.0
        self.velocity_integral = 0.0
        self.sample_count = 0
        self.earthquake_detected = False
        self.earthquake_level = "CUTREMUR INEXISTENT"
        self.wave_time = 0.0
        
        # Resetează timpii de detectare a cutremurului
        self.earthquake_detection_time = None
        self.last_earthquake_time = None
        
        # Resetează starea inițializării
        self.spi_engine_initialized = False
        self.axi_pwm_custom_initialized = False
        self.clk_generator_initialized = False
        
        if self.serial_port:
            self.serial_port.close()
            
    def read_serial_data(self):
        buffer = ""
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    lines = buffer.split('\n')
                    buffer = lines[-1]
                    
                    for line in lines[:-1]:
                        if line.strip():
                            self.process_line(line.strip())
                time.sleep(0.005)
            except Exception as e:
                print(f"Eroare serială: {e}")
                time.sleep(0.1)
                
    def process_line(self, line):
        self.terminal_lines.append(line)
        
        # Verifică mesajele de inițializare a sistemului
        if "SUCCES!" in line or "axi_adc_init" in line:
            self.spi_engine_initialized = True
            self.axi_pwm_custom_initialized = True
            self.clk_generator_initialized = True
        
        # CORECTARE: Parsează datele ADC din Vitis - ADC0 și ADC1 separat
        if "ADC0 sample:" in line:
            try:
                numbers = re.findall(r'\d+', line)
                if len(numbers) >= 6:
                    values = [int(n) for n in numbers[:6]]
                    
                    # Stochează valorile ADC0 pentru fiecare canal (mapare directă)
                    for i, val in enumerate(values):
                        self.adc0_channels[i].append(val)
                    
                    print(f"ADC0 VALUES (poziții 0-5): {values}")
                    # Debug pentru identificarea mapării corecte
                    for i, val in enumerate(values):
                        if val > 100:  # Evidențiază valorile mari
                            print(f"  >>> ADC0[{i}] = {val} (VALOARE MARE)")
                    
            except Exception as e:
                print(f"Eroare parsare ADC0: {e}")
                
        elif "ADC1 sample:" in line:
            try:
                numbers = re.findall(r'\d+', line)
                if len(numbers) >= 6:
                    values = [int(n) for n in numbers[:6]]
                    
                    # Stochează valorile ADC1 pentru fiecare canal (mapare directă)
                    for i, val in enumerate(values):
                        self.adc1_channels[i].append(val)
                    
                    print(f"ADC1 VALUES (poziții 0-5): {values}")
                    # Debug pentru identificarea mapării corecte
                    for i, val in enumerate(values):
                        if val > 100:  # Evidențiază valorile mari
                            print(f"  >>> ADC1[{i}] = {val} (VALOARE MARE)")
                    
                    # CONVERSIE VERTICALĂ CORECTATĂ: Combinează ADC0 și ADC1 pe fiecare canal
                    self.convert_adc_to_acceleration()
                    
            except Exception as e:
                print(f"Eroare parsare ADC1: {e}")

    def convert_adc_to_acceleration(self):
        """FUNCȚIE CORECTATĂ: Conversie verticală ADC0 + ADC1 -> Accelerație cu mapare corectă"""
        try:
            # Verifică dacă avem date pentru ambele ADC-uri
            if all(len(self.adc0_channels[i]) >= 0 and len(self.adc1_channels[i]) >= 0 for i in range(6)):
                
                # Obține valorile curente din ambele ADC-uri
                adc0_values = [self.adc0_channels[i][-1] if len(self.adc0_channels[i]) > 0 else 0 for i in range(6)]
                adc1_values = [self.adc1_channels[i][-1] if len(self.adc1_channels[i]) > 0 else 0 for i in range(6)]
                
                print(f"\nVALUES:")
                print(f"ADC0: {adc0_values}")
                print(f"ADC1: {adc1_values}")
                
                # CONVERSIE VERTICALĂ CORECTATĂ: Combină pe verticală (poziția cu poziția)
                # Canal 0: ADC0[0] + ADC1[0], Canal 1: ADC0[1] + ADC1[1], etc.
                for i in range(6):
                    adc0_val = adc0_values[i]
                    adc1_val = adc1_values[i]
                    
                    # CONVERSIE VERTICALĂ: Combină valorile de pe aceeași poziție
                    combined_adc = (adc0_val + adc1_val) / 2.0
                    
                    # Convertește la accelerație (0-4095 ADC -> 0-8g)
                    acceleration_g = combined_adc * self.adc_to_g_factor
                    acceleration_g = max(0.0, min(8.0, acceleration_g))
                    
                    # Actualizează accelerația pentru canalul i
                    self.channel_accelerations[i] = acceleration_g
                    
                    # Debug pentru conversie - arată maparea corectă
                    print(f"CANAL {i}: ADC0[{i}]={adc0_val}, ADC1[{i}]={adc1_val}, "
                          f"COMBINAT={(adc0_val + adc1_val)/2:.1f}, ACCELERAȚIE={acceleration_g:.3f}g")
                
                # După conversie, calculează restul parametrilor
                self.sample_count += 1
                self.calculate_axis_accelerations()
                self.calculate_richter_scale()
                self.calculate_frequency_oscilloscope()
                self.calculate_velocity_integral()
                
                # Debug final pentru toate canalele - verifică maparea
                print(f"\nSAMPLE {self.sample_count} - ACCELERAȚII FINALE (mapare corectă):")
                for i in range(6):
                    print(f"  Canal {i}: {self.channel_accelerations[i]:.3f}g")
                print(f"Axe: X={(self.channel_accelerations[0] + self.channel_accelerations[1])/2:.3f}g, "
                      f"Y={(self.channel_accelerations[2] + self.channel_accelerations[3])/2:.3f}g, "
                      f"Z={(self.channel_accelerations[4] + self.channel_accelerations[5])/2:.3f}g")
                print("-" * 80)
            
        except Exception as e:
            print(f"Eroare conversie ADC: {e}")
            import traceback
            traceback.print_exc()

    def calculate_axis_accelerations(self):
        """Calculează accelerațiile medii pentru axele X, Y, Z din canale"""
        try:
            # Axa X: Media canalelor 0 și 1
            self.axis_accelerations['X'] = (self.channel_accelerations[1] + self.channel_accelerations[1]) / 2.0
            
            # Axa Y: Media canalelor 2 și 3
            self.axis_accelerations['Y'] = (self.channel_accelerations[2] + self.channel_accelerations[3]) / 2.0
            
            # Axa Z: Media canalelor 4 și 5
            self.axis_accelerations['Z'] = (self.channel_accelerations[4] + self.channel_accelerations[5]) / 2.0
            
            # Calculează suma celor trei medii ale axelor
            self.axis_sum = self.axis_accelerations['X'] + self.axis_accelerations['Y'] + self.axis_accelerations['Z']
            
            # Media combinată pentru calculul frecvenței FFT - media celor 3 medii
            self.combined_average = self.axis_sum / 3.0
            
            # Stochează pentru grafice
            self.acceleration_data['X'].append(self.axis_accelerations['X'])
            self.acceleration_data['Y'].append(self.axis_accelerations['Y'])
            self.acceleration_data['Z'].append(self.axis_accelerations['Z'])
            
        except Exception as e:
            print(f"Eroare calculul accelerației axelor: {e}")

    def calculate_richter_scale(self):
        """FUNCȚIE CORECTATĂ: Calculează tipul cutremurului pe baza valorilor combinate ADC CONSECUTIVE pentru stabilitate - CU ANTI-NEGATIV"""
        try:
            # Analizează doar dacă avem suficiente mostre pentru stabilitate
            if self.sample_count < self.earthquake_history_length:
                return
            
            # Analizează ultimele N mostre pentru fiecare canal pentru a determina tipul stabil de cutremur
            earthquake_type_counts = {
                'DEVASTATOR': 0,
                'PUTERNIC': 0,
                'MODERAT': 0,
                'INEXISTENT': 0
            }
            
            # Verifică fiecare dintre ultimele N mostre
            for sample_idx in range(self.earthquake_history_length):
                sample_earthquake_type = self.analyze_single_sample(sample_idx)
                earthquake_type_counts[sample_earthquake_type] += 1
            
            # Determină tipul dominant de cutremur (regula majorității)
            max_count = 0
            stable_earthquake_type = 'INEXISTENT'
            
            for eq_type, count in earthquake_type_counts.items():
                if count > max_count:
                    max_count = count
                    stable_earthquake_type = eq_type
            
            # Determină nivelul anterior al cutremurului pentru detectarea schimbării 
            previous_earthquake_level = self.earthquake_level
            
            # Schimbă nivelul cutremurului doar dacă avem dovezi puternice (pragul atins)
            if max_count >= self.earthquake_consecutive_threshold:
                if stable_earthquake_type == 'DEVASTATOR':
                    self.earthquake_level = "CUTREMUR DEVASTATOR"
                    self.earthquake_detected = True
                    self.current_magnitude = self.calculate_magnitude_for_type('DEVASTATOR')
                elif stable_earthquake_type == 'PUTERNIC':
                    self.earthquake_level = "CUTREMUR PUTERNIC"
                    self.earthquake_detected = True
                    self.current_magnitude = self.calculate_magnitude_for_type('PUTERNIC')
                elif stable_earthquake_type == 'MODERAT':
                    self.earthquake_level = "CUTREMUR MODERAT"
                    self.earthquake_detected = True
                    self.current_magnitude = self.calculate_magnitude_for_type('MODERAT')
                else:
                    self.earthquake_level = "CUTREMUR INEXISTENT"
                    self.earthquake_detected = True
                    self.current_magnitude = self.calculate_magnitude_for_type('INEXISTENT')
            
            # VERIFICARE FINALĂ ANTI-NEGATIV - Garantează magnitudinea pozitivă
            if self.current_magnitude < 0:
                print(f"EROARE: Magnitudine negativă ({self.current_magnitude:.3f}) - CORECTATĂ la 0.1")
                self.current_magnitude = 0.1

            # Forțează intervalul valid 0.1-9.0
            self.current_magnitude = max(0.1, min(9.0, self.current_magnitude))
            
            # Actualizează timpul de detectare a cutremurului când nivelul se schimbă
            if (previous_earthquake_level != self.earthquake_level and 
                self.earthquake_level != "CUTREMUR INEXISTENT"):
                self.earthquake_detection_time = time.time()
                self.last_earthquake_time = time.strftime("%H:%M:%S", time.localtime())
                print(f"TIMPUL DE DETECTARE A CUTREMURULUI ACTUALIZAT: {self.last_earthquake_time} - {self.earthquake_level}")
            
            # Stochează magnitudinea pentru grafice
            self.magnitude_data.append(self.current_magnitude)
            
            # Debug la fiecare 50 de mostre pentru monitorizarea stabilității
            if self.sample_count % 50 == 0:
                print(f"\n=== DETECTARE CUTREMUR STABILĂ - Sample {self.sample_count} ===")
                print(f"Analiza ultimelor {self.earthquake_history_length} samples:")
                for eq_type, count in earthquake_type_counts.items():
                    print(f"  {eq_type}: {count}/{self.earthquake_history_length} samples")
                print(f"Tip dominant: {stable_earthquake_type} ({max_count}/{self.earthquake_history_length})")
                print(f"Threshold: {self.earthquake_consecutive_threshold}/{self.earthquake_history_length}")
                print(f"STATUS STABIL: {self.earthquake_level}")
                print(f"Magnitudine verificată: {self.current_magnitude:.3f} (interval: 0.1-9.0)")
                print("=" * 70)
                
        except Exception as e:
            print(f"Eroare calculul cutremurului stabil: {e}")
            import traceback
            traceback.print_exc()
            # SIGURANȚĂ: Setează valori sigure în caz de eroare
            self.current_magnitude = 0.1
            self.earthquake_level = "CUTREMUR INEXISTENT"
    
    def analyze_single_sample(self, sample_offset):
        """Analizează o singură mostră istorică pentru a determina tipul său de cutremur pe baza accelerațiilor combinate"""
        try:
            # Obține valorile combinate din istoricul canalelor
            current_combined_values = []
            
            for i in range(6):
                # Calculează valorile combinate istorice pentru fiecare canal
                if (len(self.adc0_channels[i]) > sample_offset and 
                    len(self.adc1_channels[i]) > sample_offset):
                    
                    adc0_historical = self.adc0_channels[i][-(sample_offset + 1)]
                    adc1_historical = self.adc1_channels[i][-(sample_offset + 1)]
                    combined_historical = (adc0_historical + adc1_historical) / 2.0
                    current_combined_values.append(combined_historical)
                else:
                    current_combined_values.append(0)
            
            # Aplică aceleași reguli dar pe valorile combinate
            channels_devastator = [val for val in current_combined_values if 1500 <= val <= 4095]
            channels_puternic = [val for val in current_combined_values if 850 <= val < 1500]
            channels_moderat = [val for val in current_combined_values if 600 <= val < 850]
            channels_inexistent = [val for val in current_combined_values if val < 600]
            
            # Returnează tipul cutremurului pentru această singură mostră
            if len(channels_devastator) >= 1:
                return 'DEVASTATOR'
            elif len(channels_puternic) >= 1:
                return 'PUTERNIC'
            elif len(channels_moderat) >= 1:
                return 'MODERAT'
            else:
                return 'INEXISTENT'
                
        except Exception as e:
            print(f"Eroare analiză mostră unică: {e}")
            return 'INEXISTENT'
    
    def calculate_magnitude_for_type(self, earthquake_type):
        """FUNCȚIE CORECTATĂ: Calculează magnitudinea pe baza tipului stabil de cutremur și valorilor recente combinate - ANTI-NEGATIV"""
        try:
            # Obține valorile combinate recente pentru calculul magnitudinii
            recent_combined_values = []
            
            for i in range(6):
                if len(self.adc0_channels[i]) > 0 and len(self.adc1_channels[i]) > 0:
                    adc0_recent = self.adc0_channels[i][-1]
                    adc1_recent = self.adc1_channels[i][-1]
                    combined_recent = (adc0_recent + adc1_recent) / 2.0
                    recent_combined_values.append(combined_recent)
                else:
                    recent_combined_values.append(0)
            
            if earthquake_type == 'DEVASTATOR':
                channels_devastator = [val for val in recent_combined_values if 1500 <= val <= 4095]
                if channels_devastator:
                    base_magnitude = 7.5
                    max_devastator = max(channels_devastator)
                    num_devastator = len(channels_devastator)
                    intensity_bonus = ((max_devastator - 1500) / 2595.0) * 1.5 + (num_devastator - 1) * 0.3
                    magnitude = base_magnitude + intensity_bonus
                    return max(7.5, min(9.0, magnitude))  # CORECTAT: forțează minimul la 7.5
                else:
                    return 7.5
                    
            elif earthquake_type == 'PUTERNIC':
                channels_puternic = [val for val in recent_combined_values if 850 <= val < 1500]
                if channels_puternic:
                    base_magnitude = 5.0
                    max_puternic = max(channels_puternic)
                    num_puternic = len(channels_puternic)
                    intensity_bonus = ((max_puternic - 850) / 650.0) * 2.5 + (num_puternic - 1) * 0.4  # CORECTAT: divizorul 650.0
                    magnitude = base_magnitude + intensity_bonus
                    return max(5.0, min(7.4, magnitude))  # CORECTAT: forțează minimul la 5.0
                else:
                    return 5.0
                    
            elif earthquake_type == 'MODERAT':
                channels_moderat = [val for val in recent_combined_values if 600 <= val < 850]  # CORECTAT: intervalul 600-850
                if channels_moderat:
                    base_magnitude = 3.0
                    max_moderat = max(channels_moderat)
                    num_moderat = len(channels_moderat)
                    intensity_bonus = ((max_moderat - 600) / 250.0) * 2.0 + (num_moderat - 1) * 0.2  # CORECTAT: divizorul și intervalul
                    magnitude = base_magnitude + intensity_bonus
                    return max(3.0, min(4.9, magnitude))  # CORECTAT: forțează minimul la 3.0
                else:
                    return 3.0
                    
            else:  # INEXISTENT
                # CORECTAT COMPLET: Pentru inexistent, returnează întotdeauna valori pozitive mici
                avg_combined = sum(recent_combined_values) / len(recent_combined_values) if recent_combined_values else 0
                
                # LOGICĂ CORECTATĂ: Inexistent = valori foarte mici, dar întotdeauna POZITIVE
                if avg_combined > 0:
                    # Mapează 0-599 ADC la 0.1-2.9 magnitudine (întotdeauna pozitiv)
                    magnitude = 0.1 + (min(avg_combined, 599) / 599.0) * 2.8
                else:
                    magnitude = 0.1  # Magnitudinea minimă când nu există date
                
                # DEBUG pentru a verifica calculele
                print(f"DEBUG INEXISTENT: avg_combined={avg_combined:.1f}, magnitude={magnitude:.3f}")
                
                return max(0.1, min(2.9, magnitude))  # GARANTEAZĂ: întotdeauna între 0.1 și 2.9
                    
        except Exception as e:
            print(f"Eroare calculul magnitudinii: {e}")
            # SIGURANȚĂ: Returnează întotdeauna o valoare pozitivă în caz de eroare
            return 0.5

    def calculate_frequency_oscilloscope(self):
        """CORECTATĂ: Frecvența CREȘTE cu intensitatea cutremurului - ca în FFT real"""
        try:
            # Calculează accelerația medie pe toate cele 3 axe (X, Y, Z)
            avg_acceleration = (self.axis_accelerations['X'] + self.axis_accelerations['Y'] + self.axis_accelerations['Z']) / 3.0
            
            # Calculează intensitatea mișcării (magnitudinea totală a accelerației)
            movement_intensity = math.sqrt(self.axis_accelerations['X']**2 + 
                                         self.axis_accelerations['Y']**2 + 
                                         self.axis_accelerations['Z']**2)
            
            # LOGICĂ CORECTĂ: Cutremure mai puternice = frecvență mai MARE (ca în FFT)
            # Similar cu graficul din imagine - crește treptat cu intensitatea
            
            if self.earthquake_level == "CUTREMUR DEVASTATOR":
                # Cutremur devastator: frecvență FOARTE MARE (50-100 Hz) - activitate seismică intensă
                base_freq = 50.0
                max_additional_freq = 50.0
                amplitude_multiplier = 8.0
                
            elif self.earthquake_level == "CUTREMUR PUTERNIC":
                # Cutremur puternic: frecvență MARE (25-60 Hz)
                base_freq = 25.0
                max_additional_freq = 35.0
                amplitude_multiplier = 6.0
                
            elif self.earthquake_level == "CUTREMUR MODERAT":
                # Cutremur moderat: frecvență MEDIE (10-35 Hz)
                base_freq = 10.0
                max_additional_freq = 25.0
                amplitude_multiplier = 4.0
                
            else:  # CUTREMUR INEXISTENT
                # Cutremur inexistent: frecvență MICĂ (1-15 Hz) - zgomot de fond minim
                base_freq = 1.0
                max_additional_freq = 14.0
                amplitude_multiplier = 1.0
            
            # CALCUL FRECVENȚĂ: Crește cu toate parametrii (ca în FFT)
            # Componenta accelerației - crește frecvența
            acceleration_component = (avg_acceleration / 8.0) * max_additional_freq * 0.6
            
            # Componenta magnitudinii - crește frecvența pentru cutremure mari
            magnitude_component = (self.current_magnitude / 9.0) * max_additional_freq * 0.8
            
            # Componenta intensității - crește frecvența cu mișcarea
            intensity_component = (movement_intensity / 8.0) * max_additional_freq * 0.4
            
            # FRECVENȚA FINALĂ: Toate componentele se ADUNĂ (mai mult = mai mare frecvență)
            self.current_frequency = base_freq + acceleration_component + magnitude_component + intensity_component
            
            # LIMITE pentru fiecare tip de cutremur
            if self.earthquake_level == "CUTREMUR DEVASTATOR":
                self.current_frequency = max(45.0, min(120.0, self.current_frequency))  # 45-120 Hz
            elif self.earthquake_level == "CUTREMUR PUTERNIC":
                self.current_frequency = max(20.0, min(80.0, self.current_frequency))   # 20-80 Hz
            elif self.earthquake_level == "CUTREMUR MODERAT":  
                self.current_frequency = max(8.0, min(50.0, self.current_frequency))    # 8-50 Hz
            else:
                self.current_frequency = max(0.5, min(20.0, self.current_frequency))    # 0.5-20 Hz
            
            # Stochează frecvența pentru grafice
            self.frequency_data.append(self.current_frequency)
            
            # Generează forma de undă ca în graficul FFT din imagine
            self.wave_time += self.dt
            
            # AMPLITUDINEA crește cu intensitatea (ca în grafic)
            amplitude_base = 0.1 + (avg_acceleration * 0.15)
            amplitude_earthquake = self.current_magnitude * 0.3 * amplitude_multiplier
            amplitude_movement = movement_intensity * 0.25
            total_amplitude = amplitude_base + amplitude_earthquake + amplitude_movement
            
            # FORMA DE UNDĂ ca în graficul FFT - complexă, cu multiple componente
            # Componenta principală
            primary_wave = total_amplitude * math.sin(2 * math.pi * self.current_frequency * self.wave_time)
            
            # Armonice care cresc cu intensitatea (ca în FFT real)
            harmonic_2 = (total_amplitude * 0.6) * math.sin(2 * math.pi * self.current_frequency * 1.3 * self.wave_time)
            harmonic_3 = (total_amplitude * 0.4) * math.sin(2 * math.pi * self.current_frequency * 1.7 * self.wave_time)
            harmonic_4 = (total_amplitude * 0.3) * math.sin(2 * math.pi * self.current_frequency * 2.1 * self.wave_time)
            
            # Componente specifice pentru cutremure (similar cu spectrul FFT)
            if self.earthquake_level == "CUTREMUR DEVASTATOR":
                # Multiple frecvențe simulate (ca în spectrul FFT complex)
                freq_1 = (total_amplitude * 1.2) * math.sin(2 * math.pi * (self.current_frequency * 0.7) * self.wave_time)
                freq_2 = (total_amplitude * 1.0) * math.sin(2 * math.pi * (self.current_frequency * 0.85) * self.wave_time)
                freq_3 = (total_amplitude * 0.8) * math.sin(2 * math.pi * (self.current_frequency * 1.15) * self.wave_time)
                freq_4 = (total_amplitude * 0.6) * math.sin(2 * math.pi * (self.current_frequency * 1.4) * self.wave_time)
                primary_wave += freq_1 + freq_2 + freq_3 + freq_4
                
            elif self.earthquake_level == "CUTREMUR PUTERNIC":
                # Componente multiple pentru puternic
                freq_1 = (total_amplitude * 0.8) * math.sin(2 * math.pi * (self.current_frequency * 0.8) * self.wave_time)
                freq_2 = (total_amplitude * 0.6) * math.sin(2 * math.pi * (self.current_frequency * 1.2) * self.wave_time)
                freq_3 = (total_amplitude * 0.4) * math.sin(2 * math.pi * (self.current_frequency * 1.6) * self.wave_time)
                primary_wave += freq_1 + freq_2 + freq_3
                
            elif self.earthquake_level == "CUTREMUR MODERAT":
                # Componente moderate
                freq_1 = (total_amplitude * 0.5) * math.sin(2 * math.pi * (self.current_frequency * 0.9) * self.wave_time)
                freq_2 = (total_amplitude * 0.3) * math.sin(2 * math.pi * (self.current_frequency * 1.3) * self.wave_time)
                primary_wave += freq_1 + freq_2
            
            # Adaugă zgomot aleator pentru realism (ca în FFT real)
            if movement_intensity > 0.2:
                noise_amplitude = min(0.2, movement_intensity * 0.1) * amplitude_multiplier * 0.5
                # Zgomot în jurul frecvenței principale
                noise_freq = self.current_frequency + (math.sin(self.wave_time * 0.3) * self.current_frequency * 0.1)
                noise = noise_amplitude * math.sin(2 * math.pi * noise_freq * self.wave_time)
                primary_wave += noise
            
            # Combină toate componentele (ca în analiza FFT complexă)
            final_wave_value = primary_wave + harmonic_2 + harmonic_3 + harmonic_4
            
            # Stochează datele formei de undă pentru afișajul osciloscopului
            wave_data_point = {
                'time': self.wave_time,
                'amplitude': final_wave_value,
                'frequency': self.current_frequency,
                'magnitude': self.current_magnitude,
                'avg_acceleration': avg_acceleration,
                'movement_intensity': movement_intensity,
                'earthquake_level': self.earthquake_level
            }
            
            self.oscilloscope_wave_data.append(wave_data_point)
            
            # Actualizează media combinată pentru afișaj
            self.combined_average = avg_acceleration
            
            # Debug pentru frecvența CORECTĂ (crește cu intensitatea)
            if self.sample_count % 50 == 0:
                print(f"\n=== DEBUG FRECVENȚĂ FFT CORECTĂ ===")
                print(f"Tip cutremur: {self.earthquake_level}")
                print(f"Magnitudine: {self.current_magnitude:.3f}")
                print(f"Media acceleratii: {avg_acceleration:.3f}g")
                print(f"Intensitate: {movement_intensity:.3f}g")
                print(f"Frecventa de baza: {base_freq:.1f}Hz")
                print(f"Componenta accelerație: +{acceleration_component:.1f}Hz")
                print(f"Componenta magnitudine: +{magnitude_component:.1f}Hz")
                print(f"Componenta intensitate: +{intensity_component:.1f}Hz")
                print(f"FRECVENTA FINALA: {self.current_frequency:.2f}Hz")
                print(f"Amplitudine: {total_amplitude:.3f}")
                print(f"LOGICĂ CORECTĂ: Mai mult cutremur = frecvență mai MARE (ca în FFT)")
                if self.earthquake_level == "CUTREMUR DEVASTATOR":
                    print(f"  → Interval: 45-120 Hz")
                elif self.earthquake_level == "CUTREMUR PUTERNIC":
                    print(f"  → Interval: 20-80 Hz")
                elif self.earthquake_level == "CUTREMUR MODERAT":
                    print(f"  → Interval: 8-50 Hz")
                else:
                    print(f"  → Interval: 0.5-20 Hz")
                print("=" * 60)
            
        except Exception as e:
            print(f"Eroare calculul frecvenței FFT: {e}")
            self.current_frequency = 5.0  # Frecvență implicită pentru inexistent

    def calculate_velocity_integral(self):
        """Calculează viteza undelor seismice ca integrala sumei accelerațiilor X+Y+Z"""
        try:
            # Suma tuturor accelerațiilor axelor (X + Y + Z)
            total_acceleration_sum = self.axis_accelerations['X'] + self.axis_accelerations['Y'] + self.axis_accelerations['Z']
            
            # Integrează suma accelerațiilor pentru a obține viteza
            # v = ∫(a_x + a_y + a_z) dt
            self.velocity_integral += total_acceleration_sum * self.dt
            
            # Aplică amortizarea ușoară pentru a preveni creșterea infinită, dar păstrează capacitatea de răspuns
            self.velocity_integral *= 0.995  # Amortizare mai mică pentru răspuns mai bun
            
            # Convertește la viteza seismică - scalare mai sensibilă
            # Când placa se mișcă mai tare, viteza ar trebui să fie semnificativ mai mare
            velocity_magnitude = abs(self.velocity_integral)
            
            # Factor de scală - fă-l mai receptiv la mișcarea plăcii
            scale_factor = 0.1  # Mărit pentru vizibilitate mai bună
            self.current_velocity = velocity_magnitude * scale_factor
            
            # Scalare adițională pe baza intensității accelerației curente
            acceleration_intensity = math.sqrt(self.axis_accelerations['X']**2 + 
                                             self.axis_accelerations['Y']**2 + 
                                             self.axis_accelerations['Z']**2)
            
            # Mărește viteza când accelerația este mare (placa se mișcă tare)
            if acceleration_intensity > 1.0:
                intensity_boost = (acceleration_intensity - 1.0) * 2.0
                self.current_velocity += intensity_boost
            
            # Limitează viteza la interval seismic rezonabil dar permite valori mai mari
            self.current_velocity = min(self.current_velocity, 50.0)  # Maximul mărit
            
            # Debug calculul vitezei
            if self.sample_count % 200 == 0:
                print(f"Debug Viteza - Suma accelerații: {total_acceleration_sum:.3f}g")
                print(f"Debug Viteza - Integrala: {self.velocity_integral:.6f}")
                print(f"Debug Viteza - Intensitate: {acceleration_intensity:.3f}g")
                print(f"Debug Viteza - Viteza finală: {self.current_velocity:.3f} m/s")
            
        except Exception as e:
            print(f"Eroare calculul vitezei: {e}")
            self.current_velocity = 0.0
    
    def draw_box(self, rect, title, content=None, clickable=True, color=None):
        color = color or self.WHITE
        pygame.draw.rect(self.screen, color, rect)
        pygame.draw.rect(self.screen, self.NAVY, rect, 3)
        
        # Stilizare curată a titlului fără efecte de umbră
        lines = title.split('\n')
        start_y = rect.y + 20  # Poziționare ușor ajustată
        
        for i, line in enumerate(lines):
            if line.strip():  # Procesează doar liniile non-goale
                # Desenează titlul principal cu culoare curată - fără umbră
                main_color = (25, 25, 112)  # Albastru de miezul nopții - la fel ca titlul principal
                text = self.box_font.render(line, True, main_color)
                text_rect = text.get_rect(center=(rect.centerx, start_y + i * 30))
                self.screen.blit(text, text_rect)
        
        if content:
            # Stilizare curată a conținutului fără umbră
            content_color = (0, 0, 180)  # Albastru mai închis pentru contrast mai bun
            content_text = self.value_font.render(content, True, content_color)
            content_rect = content_text.get_rect(center=(rect.centerx, rect.centery + 25))
            self.screen.blit(content_text, content_rect)
        
        if clickable:
            # Stilizare îmbunătățită a butonului
            btn_rect = pygame.Rect(rect.centerx - 55, rect.bottom - 35, 110, 25)  # Puțin mai lat
            
            # Efect de gradient pentru buton
            pygame.draw.rect(self.screen, (40, 40, 160), btn_rect)  # Bază mai închisă
            pygame.draw.rect(self.screen, self.NAVY, btn_rect)
            pygame.draw.rect(self.screen, (80, 80, 200), btn_rect, 1)  # Margine mai deschisă
            
            # Text îmbunătățit pentru buton
            btn_font = pygame.font.Font(None, 20)
            btn_font.set_bold(True)
            btn_text = btn_font.render("Vizualizare", True, self.WHITE)
            btn_text_rect = btn_text.get_rect(center=btn_rect.center)
            self.screen.blit(btn_text, btn_text_rect)
    
    def draw_acceleration_graph(self, surface, data, color, current_val, axis_name):
        w, h = surface.get_size()
        surface.fill(self.WHITE)
        pygame.draw.rect(surface, self.GRAY, (0, 0, w, h), 2)
        
        # Grilă
        for i in range(1, 11):
            x = i * w // 11
            pygame.draw.line(surface, (220, 220, 220), (x, 0), (x, h), 1)
        for i in range(1, 9):  # Linii orizontale pentru scara 0-8g
            y = i * h // 9
            pygame.draw.line(surface, (220, 220, 220), (0, y), (w, y), 1)
        
        # Titlu care arată valoarea accelerației curente
        text_surface = pygame.font.Font(None, 22)
        text_surface.set_bold(True)
        val_text = f"Accelerația pe axa {axis_name}: {current_val:.2f}g"
        text = text_surface.render(val_text, True, color)
        surface.blit(text, (10, 10))
        
        # Etichete de scală pentru intervalul 0g la 8g (doar pozitive) - mutate departe de marginea stângă
        scale_values = [8, 7, 6, 5, 4, 3, 2, 1, 0]
        
        for i, val in enumerate(scale_values):
            if val == 0:  # Omite eticheta 0g pentru a o elimina din partea stângă
                continue
            y_pos = 35 + i * (h - 70) // (len(scale_values) - 1)
            scale_text = self.small_font.render(f"{val}g", True, color)
            surface.blit(scale_text, (w - 35, y_pos - 8))
        
        # Linia zero (0g) - în partea de jos, fără etichetă
        zero_y = h - 35
        pygame.draw.line(surface, self.DARK_GREEN, (50, zero_y), (w - 50, zero_y), 2)
        
        # Desenează semnal de accelerație variabilă cu vârfuri
        if len(data) > 1:
            points = list(data)[-200:]  # Ultimele 200 de mostre pentru semnal lin
            if points:
                graph_points = []
                
                for i, val in enumerate(points):
                    x = int(i * (w - 100) / max(len(points) - 1, 1)) + 50
                    
                    # Mapează 0g la 8g la înălțimea graficului (inversat - 0g jos, 8g sus)
                    y = int((8.0 - val) / 8.0 * (h - 70)) + 35
                    y = max(35, min(h - 35, y))
                    graph_points.append((x, y))
                
                # Desenează semnalul cu vârfuri vizibile când accelerația crește
                if len(graph_points) > 1:
                    pygame.draw.lines(surface, color, False, graph_points, 3)
                    
                    # Adaugă vârfuri/puncte maxime pentru valori mai mari ale accelerației
                    for i, (x, y) in enumerate(graph_points):
                        acc_val = points[i] if i < len(points) else 0
                        if acc_val > 2.0:  # Adaugă vârfuri pentru accelerația > 2g
                            spike_height = int(min(20, acc_val * 3))
                            pygame.draw.line(surface, color, (x, y), (x, y - spike_height), 2)
                    
                    # Evidențiază punctul curent
                    if graph_points:
                        current_point = graph_points[-1]
                        pygame.draw.circle(surface, color, current_point, 7)
                        pygame.draw.circle(surface, self.WHITE, current_point, 5)
                        pygame.draw.circle(surface, color, current_point, 3)
                        
                        # Eticheta valorii curente cu fundal
                        val_label = f"{current_val:.2f}g"
                        label_font = pygame.font.Font(None, 16)
                        label_text = label_font.render(val_label, True, color)
                        label_x = max(5, min(w-60, current_point[0] - 25))
                        label_y = max(5, current_point[1] - 25)
                        
                        # Fundal pentru etichetă
                        label_rect = pygame.Rect(label_x - 2, label_y - 2, 
                                               label_text.get_width() + 4, label_text.get_height() + 4)
                        pygame.draw.rect(surface, self.WHITE, label_rect)
                        pygame.draw.rect(surface, color, label_rect, 1)
                        surface.blit(label_text, (label_x, label_y))
        else:
            # Când nu există date, arată linie plată la 0g
            baseline_y = h - 35  # Poziția 0g
            
            # Desenează linia de bază orizontală la 0g
            pygame.draw.line(surface, color, (50, baseline_y), (w - 50, baseline_y), 2)
            pygame.draw.circle(surface, color, (w - 70, baseline_y), 4)
    
    def draw_acceleration_window(self):
        window = pygame.Surface((1100, 750))  # Făcut mai înalt pentru textul de stare
        window.fill(self.WHITE)
        
        title_font = pygame.font.Font(None, 42)
        title_font.set_bold(True)
        title = title_font.render("Accelerația Seismică", True, self.NAVY)
        title_rect = title.get_rect(center=(550, 30))  # Centrat la jumătatea lățimii
        window.blit(title, title_rect)
        
        # Trei grafice pentru mediile axelor X, Y, Z
        graph_w, graph_h = 1000, 180
        positions = [(50, 60), (50, 270), (50, 480)]
        axes_data = [
            ('X', self.RED, self.axis_accelerations['X'], self.acceleration_data['X']),
            ('Y', self.GREEN, self.axis_accelerations['Y'], self.acceleration_data['Y']),
            ('Z', self.BLUE, self.axis_accelerations['Z'], self.acceleration_data['Z'])
        ]
        
        for i, ((x, y), (axis, color, current, data)) in enumerate(zip(positions, axes_data)):
            title_font = pygame.font.Font(None, 28)
            title_font.set_bold(True)
            title_text = title_font.render(f"Accelerația pe axa {axis}", True, color)
            window.blit(title_text, (x, y - 25))
            
            graph_surface = pygame.Surface((graph_w, graph_h))
            self.draw_acceleration_graph(graph_surface, data, color, current, axis)
            window.blit(graph_surface, (x, y))
            
            pygame.draw.rect(window, self.GRAY, (x, y, graph_w, graph_h), 3)
        
        # Afișajul stării mutat sub graficul axei Z
        status_text = f"Accelerații calculate din combinația sample ADC0+ADC1 (0-4095 → 0-8g)"
        status_color = self.GREEN if self.connected else self.RED
        status_font = pygame.font.Font(None, 20)
        status_render = status_font.render(status_text, True, status_color)
        window.blit(status_render, (50, 680))
        
        close_rect = pygame.Rect(1050, 20, 40, 30)
        pygame.draw.rect(window, self.RED, close_rect)
        close_text = self.small_font.render("X", True, self.WHITE)
        window.blit(close_text, (close_rect.centerx - 5, close_rect.centery - 10))
        
        self.screen.blit(window, (50, 25))
        return close_rect.move(50, 25)
    
    def draw_magnitude_window(self):
        window = pygame.Surface((750, 550))
        window.fill(self.WHITE)
        
        title_font = pygame.font.Font(None, 42)
        title_font.set_bold(True)
        title = title_font.render("Scara Richter", True, self.NAVY)
        window.blit(title, (20, 20))
        
        # Informații despre calculul din valorile combinate ADC 
        info_font = pygame.font.Font(None, 18)
        info_text = info_font.render("Bazată pe sample ADC", True, self.NAVY)
        window.blit(info_text, (20, 50))
        
        # Afișajul stării cutremurului cu stilizare curată, statică - FĂRĂ PULSARE
        status_font = pygame.font.Font(None, 26)
        status_font.set_bold(True)
        
        # Formatează textul stării cu efecte vizuale statice 
        if "DEVASTATOR" in self.earthquake_level:
            status_text = "STATUS: CUTREMUR DEVASTATOR!!"
            status_color = self.DARK_RED
            bg_color = (255, 200, 200)  # Culoare de fundal statică
        elif "PUTERNIC" in self.earthquake_level:
            status_text = "STATUS: CUTREMUR PUTERNIC!!"
            status_color = self.BRIGHT_RED
            bg_color = (255, 220, 180)
        elif "MODERAT" in self.earthquake_level:
            status_text = "STATUS: CUTREMUR MODERAT!"
            status_color = self.BRIGHT_ORANGE
            bg_color = (255, 255, 200)
        else:
            status_text = "STATUS: CUTREMUR INEXISTENT!"
            status_color = self.GREEN
            bg_color = (200, 255, 200)
        
        # Afișaj static al stării cu fundal - fără animații
        status_bg_rect = pygame.Rect(15, 72, 720, 35)
        pygame.draw.rect(window, bg_color, status_bg_rect)
        pygame.draw.rect(window, status_color, status_bg_rect, 3)
        
        status_render = status_font.render(status_text, True, status_color)
        status_rect = status_render.get_rect(center=(375, 90))
        window.blit(status_render, status_rect)
        
        # Graficul îmbunătățit al magnitudinii Richter cu modelul de grilă pătrată
        graph_surface = pygame.Surface((700, 280))
        graph_surface.fill(self.WHITE)
        pygame.draw.rect(graph_surface, self.GRAY, (0, 0, 700, 280), 2)
        
        # Desenează modelul de grilă pătrată
        square_size = 20
        grid_color = (240, 240, 240)
        for x in range(30, 700, square_size):
            for y in range(25, 280, square_size):
                pygame.draw.rect(graph_surface, grid_color, (x, y, square_size, square_size), 1)
        
        # Desenează liniile scalei magnitudinii 0-9 cu stilizare îmbunătățită
        scale_font = pygame.font.Font(None, 22)
        scale_font.set_bold(True)
        for i in range(10):
            y_pos = 25 + i * 28
            
            # Culori alternate pentru vizibilitate mai bună
            if i % 2 == 0:
                scale_line_color = (200, 200, 200)
                scale_bg_color = (250, 250, 250)
            else:
                scale_line_color = (220, 220, 220)
                scale_bg_color = (245, 245, 245)
            
            # Dungă de fundal
            pygame.draw.rect(graph_surface, scale_bg_color, (30, y_pos - 14, 670, 28))
            pygame.draw.line(graph_surface, scale_line_color, (30, y_pos), (700, y_pos), 2)
            
            scale_value = 9 - i
            scale_text = scale_font.render(f"{scale_value}.0", True, self.NAVY)
            
            # Fundal pentru textul scalei
            text_bg = pygame.Rect(5, y_pos - 10, 25, 20)
            pygame.draw.rect(graph_surface, self.WHITE, text_bg)
            pygame.draw.rect(graph_surface, self.NAVY, text_bg, 1)
            
            graph_surface.blit(scale_text, (7, y_pos - 8))
        
        # Desenează semnalul îmbunătățit al magnitudinii cu vizualizare frumoasă a cutremurului
        if len(self.magnitude_data) > 1:
            points = list(self.magnitude_data)[-400:]
            
            if len(points) > 1:
                magnitude_points = []
                graph_width = 670
                
                # Creează forma de undă seismică îmbunătățită cu efecte bazate pe stare
                for i, magnitude in enumerate(points):
                    x = 30 + int(i * graph_width / max(len(points) - 1, 1))
                    
                    # Poziția Y de bază pentru magnitudine
                    base_y = 277 - int((magnitude / 9.0) * 252)
                    base_y = max(25, min(277, base_y))
                    
                    # Oscilații îmbunătățite pe baza stării cutremurului
                    if "DEVASTATOR" in self.earthquake_level:
                        # Oscilații violente pentru cutremure devastatoare
                        oscillation_amplitude = min(60, magnitude * 20)
                        oscillation_frequency = 0.5 + magnitude * 0.3
                        
                        # Multiple componente de undă pentru haos
                        wave_offset = math.sin(i * oscillation_frequency) * oscillation_amplitude
                        wave_offset += math.sin(i * oscillation_frequency * 2.5) * (oscillation_amplitude * 0.6)
                        wave_offset += math.sin(i * oscillation_frequency * 4.0) * (oscillation_amplitude * 0.3)
                        
                        final_y = int(base_y + wave_offset)
                        
                    elif "PUTERNIC" in self.earthquake_level:
                        # Oscilații puternice
                        oscillation_amplitude = min(45, magnitude * 15)
                        oscillation_frequency = 0.4 + magnitude * 0.25
                        
                        wave_offset = math.sin(i * oscillation_frequency) * oscillation_amplitude
                        wave_offset += math.sin(i * oscillation_frequency * 1.8) * (oscillation_amplitude * 0.4)
                        
                        final_y = int(base_y + wave_offset)
                        
                    elif "MODERAT" in self.earthquake_level:
                        # Oscilații moderate
                        oscillation_amplitude = min(30, magnitude * 10)
                        oscillation_frequency = 0.3 + magnitude * 0.2
                        
                        wave_offset = math.sin(i * oscillation_frequency) * oscillation_amplitude
                        wave_offset += math.sin(i * oscillation_frequency * 1.3) * (oscillation_amplitude * 0.3)
                        
                        final_y = int(base_y + wave_offset)
                        
                    else:
                        # Oscilații mici de linie de bază pentru inexistent
                        small_wave = math.sin(i * 0.15) * 8
                        final_y = int(base_y + small_wave)
                    
                    final_y = max(25, min(277, final_y))
                    magnitude_points.append((x, final_y))
                
                if len(magnitude_points) > 1:
                    # Colorare dinamică îmbunătățită și grosime
                    if "DEVASTATOR" in self.earthquake_level:
                        line_color = (180, 0, 0)
                        line_thickness = 6
                        glow_color = (255, 100, 100)
                    elif "PUTERNIC" in self.earthquake_level:
                        line_color = (200, 50, 0)
                        line_thickness = 5
                        glow_color = (255, 150, 50)
                    elif "MODERAT" in self.earthquake_level:
                        line_color = (200, 140, 0)
                        line_thickness = 4
                        glow_color = (255, 200, 100)
                    else:
                        line_color = (0, 120, 80)
                        line_thickness = 3
                        glow_color = (100, 200, 150)
                    
                    # Desenează efectul de strălucire primul (linie mai lată, mai deschisă)
                    if len(magnitude_points) > 1:
                        pygame.draw.lines(graph_surface, glow_color, False, magnitude_points, line_thickness + 2)
                    
                    # Desenează forma de undă seismică principală
                    pygame.draw.lines(graph_surface, line_color, False, magnitude_points, line_thickness)
                    
                    # Adaugă vârfuri îmbunătățite pentru cutremure puternice
                    if "DEVASTATOR" in self.earthquake_level:
                        for i in range(0, len(magnitude_points), 3):
                            if i < len(magnitude_points):
                                x, y = magnitude_points[i]
                                spike_height = int(25 + self.current_magnitude * 8)
                                pygame.draw.line(graph_surface, (255, 0, 0), (x, y), (x, y - spike_height), 4)
                                pygame.draw.line(graph_surface, (255, 0, 0), (x, y), (x, y + spike_height), 4)
                    elif "PUTERNIC" in self.earthquake_level:
                        for i in range(0, len(magnitude_points), 5):
                            if i < len(magnitude_points):
                                x, y = magnitude_points[i]
                                spike_height = int(18 + self.current_magnitude * 5)
                                pygame.draw.line(graph_surface, (255, 100, 0), (x, y), (x, y - spike_height), 3)
                    
                    # Indicator îmbunătățit al punctului curent - fix, fără pulsare
                    if magnitude_points:
                        current_point = magnitude_points[-1]
                        point_size = 8 + int(self.current_magnitude * 2)
                        
                        # Indicator static fără efect de pulsare
                        pygame.draw.circle(graph_surface, line_color, current_point, point_size)
                        pygame.draw.circle(graph_surface, self.WHITE, current_point, point_size - 3)
                        pygame.draw.circle(graph_surface, line_color, current_point, point_size - 6)
        
        window.blit(graph_surface, (25, 115))
        
        # Afișaj static al valorii magnitudinii - poziționat corespunzător în fereastră
        mag_font = pygame.font.Font(None, 32)
        mag_font.set_bold(True)
        mag_text = f"Magnitudine: {self.current_magnitude:.2f}/9"
        mag_color = status_color
        
        # Fundal pentru afișajul magnitudinii - static, fără animații
        mag_bg_rect = pygame.Rect(200, 405, 350, 35)  # Mai lat și mai bine poziționat
        pygame.draw.rect(window, bg_color, mag_bg_rect)
        pygame.draw.rect(window, mag_color, mag_bg_rect, 2)
        
        mag_render = mag_font.render(mag_text, True, mag_color)
        mag_rect = mag_render.get_rect(center=(375, 422))  # Centrat în fundal
        window.blit(mag_render, mag_rect)
        
        # Informații despre reguli actualizate cu intervalele combinate
        info_lines = [
            f"REGULI: <600 = INEXISTENT (0.1-2.9)",
            f"600-849 = MODERAT (3.0-4.9), 850-1499 = PUTERNIC (5.0-7.4)",
            f"1500-4095 = DEVASTATOR (7.5-9.0)"
        ]
        
        y_offset = 460
        info_font_small = pygame.font.Font(None, 20)
        for line in info_lines:
            text_surface = info_font_small.render(line, True, self.NAVY)
            window.blit(text_surface, (25, y_offset))
            y_offset += 25
        
        close_rect = pygame.Rect(700, 20, 40, 30)
        pygame.draw.rect(window, self.RED, close_rect)
        close_text = self.small_font.render("X", True, self.WHITE)
        window.blit(close_text, (close_rect.centerx - 5, close_rect.centery - 10))
        
        self.screen.blit(window, (225, 125))
        return close_rect.move(225, 125)
    
    def draw_frequency_window(self):
        """ACTUALIZAT: Afișaj frumos al osciloscopului cu colorare dinamică bazată pe tipul cutremurului"""
        window = pygame.Surface((1000, 700))
        window.fill(self.WHITE)
        
        title_font = pygame.font.Font(None, 42)
        title_font.set_bold(True)
        title = title_font.render("OSCILOSCOP SEISMIC", True, self.NAVY)
        window.blit(title, (20, 20))
        
        # Zona de afișaj a osciloscopului cu fundal colorat dinamic
        scope_surface = pygame.Surface((950, 450))
        
        # COLORARE DINAMICĂ A FUNDALULUI PE BAZA TIPULUI CUTREMURULUI
        if self.earthquake_level == "CUTREMUR DEVASTATOR":
            scope_bg_color = (20, 0, 0)  # Roșu foarte închis pentru devastator
            grid_primary_color = (120, 0, 0)  # Roșu pentru grila principală
            grid_secondary_color = (60, 0, 0)  # Roșu mai închis pentru grila secundară
            center_line_color = (255, 50, 50)  # Roșu strălucitor pentru linia centrală
        elif self.earthquake_level == "CUTREMUR PUTERNIC":
            scope_bg_color = (15, 5, 0)  # Portocaliu foarte închis pentru puternic
            grid_primary_color = (120, 60, 0)  # Portocaliu pentru grila principală
            grid_secondary_color = (60, 30, 0)  # Portocaliu mai închis pentru grila secundară
            center_line_color = (255, 150, 0)  # Portocaliu strălucitor pentru linia centrală
        elif self.earthquake_level == "CUTREMUR MODERAT":
            scope_bg_color = (10, 10, 0)  # Galben foarte închis pentru moderat
            grid_primary_color = (120, 120, 0)  # Galben pentru grila principală
            grid_secondary_color = (60, 60, 0)  # Galben mai închis pentru grila secundară
            center_line_color = (255, 255, 50)  # Galben strălucitor pentru linia centrală
        else:  # CUTREMUR INEXISTENT
            scope_bg_color = (0, 10, 0)  # Verde foarte închis pentru inexistent (implicit)
            grid_primary_color = (0, 120, 0)  # Verde pentru grila principală
            grid_secondary_color = (0, 60, 0)  # Verde mai închis pentru grila secundară
            center_line_color = (0, 180, 0)  # Verde strălucitor pentru linia centrală
        
        scope_surface.fill(scope_bg_color)
        pygame.draw.rect(scope_surface, grid_primary_color, (0, 0, 950, 450), 3)
        
        # Grila osciloscopului - stil profesional cu colorare dinamică
        
        # Linii majore verticale ale grilei (diviziuni de timp)
        for i in range(0, 951, 95):  # 10 diviziuni majore
            color = grid_primary_color if i % 190 == 0 else grid_secondary_color
            line_width = 2 if i % 190 == 0 else 1
            pygame.draw.line(scope_surface, color, (i, 0), (i, 450), line_width)
        
        # Linii minore verticale ale grilei
        for i in range(0, 951, 19):  # 50 diviziuni minore
            if i % 95 != 0:  # Nu suprapune liniile majore
                minor_color = (grid_secondary_color[0]//2, grid_secondary_color[1]//2, grid_secondary_color[2]//2)
                pygame.draw.line(scope_surface, minor_color, (i, 0), (i, 450), 1)
        
        # Linii majore orizontale ale grilei (diviziuni de amplitudine)
        center_y = 225  # Linia centrală
        for i in range(0, 451, 45):  # 10 diviziuni majore
            if i == center_y:
                color = center_line_color  # Linia centrală cu culoare specială
                line_width = 3
            elif i % 90 == 0:
                color = grid_primary_color
                line_width = 2
            else:
                color = grid_secondary_color
                line_width = 1
            pygame.draw.line(scope_surface, color, (0, i), (950, i), line_width)
        
        # Linii minore orizontale ale grilei
        for i in range(0, 451, 9):  # 50 diviziuni minore
            if i % 45 != 0:  # Nu suprapune liniile majore
                minor_color = (grid_secondary_color[0]//2, grid_secondary_color[1]//2, grid_secondary_color[2]//2)
                pygame.draw.line(scope_surface, minor_color, (0, i), (950, i), 1)
        
        # Desenează forma de undă seismică reală din datele osciloscopului
        if len(self.oscilloscope_wave_data) > 10:
            wave_points = list(self.oscilloscope_wave_data)[-950:]  # Ultimele 950 puncte pentru lățimea completă
            
            if len(wave_points) > 1:
                scope_points = []
                
                # Calculează scalarea dinamică a amplitudinii pe baza condițiilor curente
                max_amplitude = max(abs(point['amplitude']) for point in wave_points) if wave_points else 1.0
                if max_amplitude < 0.1:
                    max_amplitude = 0.1  # Previne împărțirea la zero
                
                # Factor de scală pentru afișajul osciloscopului (centru ± interval amplitudine)
                amplitude_scale = 180.0 / max_amplitude  # Folosește 180 pixeli pentru ±max_amplitude
                
                for i, wave_point in enumerate(wave_points):
                    x = int(i * 950 / max(len(wave_points) - 1, 1))
                    
                    # Mapează amplitudinea la poziția Y (linia centrală ± amplitudine scalată)
                    amplitude = wave_point['amplitude']
                    y = int(center_y - (amplitude * amplitude_scale))
                    y = max(10, min(440, y))  # Păstrează în limite
                    
                    scope_points.append((x, y))
                
                # Desenează forma de undă cu colorare dinamică pe baza tipului cutremurului
                if len(scope_points) > 1:
                    # COLORARE DINAMICĂ A URMEI PE BAZA TIPULUI CUTREMURULUI
                    if self.earthquake_level == "CUTREMUR DEVASTATOR":
                        trace_color = (255, 100, 100)  # Roșu strălucitor pentru devastator
                        trace_width = 4
                        glow_color = (180, 50, 50)
                    elif self.earthquake_level == "CUTREMUR PUTERNIC":
                        trace_color = (255, 200, 0)    # Portocaliu pentru puternic
                        trace_width = 3
                        glow_color = (180, 140, 0)
                    elif self.earthquake_level == "CUTREMUR MODERAT":
                        trace_color = (255, 255, 0)    # Galben pentru moderat
                        trace_width = 2
                        glow_color = (180, 180, 0)
                    else:  # CUTREMUR INEXISTENT
                        trace_color = (0, 255, 100)    # Verde pentru slab/inexistent
                        trace_width = 2
                        glow_color = (0, 180, 70)
                    
                    # Desenează efectul de strălucire care se potrivește cu culoarea cutremurului
                    if self.current_magnitude > 3.0:
                        # Desenează urme ușor deplasate pentru efectul de strălucire
                        offset_points_up = [(x, max(10, y-1)) for x, y in scope_points]
                        offset_points_down = [(x, min(440, y+1)) for x, y in scope_points]
                        pygame.draw.lines(scope_surface, glow_color, False, offset_points_up, 1)
                        pygame.draw.lines(scope_surface, glow_color, False, offset_points_down, 1)
                    
                    # Desenează urma principală a formei de undă
                    pygame.draw.lines(scope_surface, trace_color, False, scope_points, trace_width)
                    
                    # Evidențiază punctul curent (pata fasciculului) cu culoarea corespunzătoare
                    if scope_points:
                        current_point = scope_points[-1]
                        beam_size = 6 + int(self.current_magnitude * 2)
                        
                        # Desenează pata fasciculului cu strălucire în culoarea cutremurului
                        pygame.draw.circle(scope_surface, (255, 255, 255), current_point, beam_size)
                        pygame.draw.circle(scope_surface, trace_color, current_point, beam_size - 2)
                        pygame.draw.circle(scope_surface, (255, 255, 255), current_point, beam_size - 4)
                        
                        # Adaugă urma de persistență (strălucire reziduală) pentru ultimele puncte
                        trail_length = min(20, len(scope_points))
                        for j in range(trail_length):
                            if j < len(scope_points):
                                trail_point = scope_points[-(j+1)]
                                trail_alpha = (trail_length - j) / trail_length
                                trail_size = int(3 * trail_alpha)
                                if trail_size > 0:
                                    # Culoarea urmei se potrivește cu culoarea cutremurului
                                    trail_color = (int(trace_color[0] * trail_alpha * 0.5),
                                                 int(trace_color[1] * trail_alpha * 0.5),
                                                 int(trace_color[2] * trail_alpha * 0.5))
                                    pygame.draw.circle(scope_surface, trail_color, trail_point, trail_size)
        
        # Scara de timp a osciloscopului (jos) - colorată conform tipului cutremurului
        time_scale_font = pygame.font.Font(None, 14)
        time_scale_font.set_bold(True)
        time_labels = ["0ms", "10ms", "20ms", "30ms", "40ms", "50ms", "60ms", "70ms", "80ms", "90ms", "100ms"]
        
        # Culoarea textului scalei se potrivește cu grila principală
        scale_text_color = grid_primary_color
        for i, time_label in enumerate(time_labels):
            x_pos = i * 95
            if x_pos <= 950:
                label_text = time_scale_font.render(time_label, True, scale_text_color)
                scope_surface.blit(label_text, (x_pos - 15, 430))
        
        # Scara de amplitudine a osciloscopului (partea stângă) - colorată conform tipului cutremurului
        amplitude_labels = ["+5", "+4", "+3", "+2", "+1", "0", "-1", "-2", "-3", "-4", "-5"]
        for i, amp_label in enumerate(amplitude_labels):
            y_pos = i * 45 + 10
            if y_pos <= 450:
                # Evidențiază linia centrală (0) cu culoarea specială
                if amp_label == "0":
                    label_color = center_line_color
                else:
                    label_color = scale_text_color
                label_text = time_scale_font.render(amp_label, True, label_color)
                scope_surface.blit(label_text, (5, y_pos - 7))
        
        window.blit(scope_surface, (25, 80))
        
        # Panoul de control al osciloscopului (ca osciloscopul real) - colorat conform tipului cutremurului
        control_panel = pygame.Rect(25, 540, 950, 120)
        
        # Fundalul panoului de control se potrivește cu tema cutremurului
        if self.earthquake_level == "CUTREMUR DEVASTATOR":
            control_bg_color = (40, 20, 20)
            control_border_color = (120, 0, 0)
            readout_color = (255, 150, 150)
        elif self.earthquake_level == "CUTREMUR PUTERNIC":
            control_bg_color = (40, 30, 20)
            control_border_color = (120, 60, 0)
            readout_color = (255, 200, 100)
        elif self.earthquake_level == "CUTREMUR MODERAT":
            control_bg_color = (40, 40, 20)
            control_border_color = (120, 120, 0)
            readout_color = (255, 255, 150)
        else:  # CUTREMUR INEXISTENT
            control_bg_color = (30, 30, 30)
            control_border_color = (0, 120, 0)
            readout_color = (150, 255, 150)
            
        pygame.draw.rect(window, control_bg_color, control_panel)
        pygame.draw.rect(window, control_border_color, control_panel, 3)
        
        # Afișaje de control în stilul osciloscopului cu culori dinamice
        readout_font = pygame.font.Font(None, 18)
        readout_font.set_bold(True)
        
        # Coloana stângă - Măsurători principale
        freq_text = readout_font.render(f"FREQ: {self.current_frequency:.2f} Hz", True, readout_color)
        window.blit(freq_text, (35, 550))
        
        magnitude_text = readout_font.render(f"MAGNITUDE: {self.current_magnitude:.2f}", True, readout_color)
        window.blit(magnitude_text, (35, 570))
        
        avg_accel_text = readout_font.render(f"AVG ACCEL: {self.combined_average:.2f}g", True, readout_color)
        window.blit(avg_accel_text, (35, 590))
        
        level_text = readout_font.render(f"EARTHQUAKE: {self.earthquake_level}", True, readout_color)
        window.blit(level_text, (35, 610))
        
        # Coloana centrală - Detalii axe
        axis_x_text = readout_font.render(f"AXIS X: {self.axis_accelerations['X']:.2f}g", True, readout_color)
        window.blit(axis_x_text, (300, 550))
        
        axis_y_text = readout_font.render(f"AXIS Y: {self.axis_accelerations['Y']:.2f}g", True, readout_color)
        window.blit(axis_y_text, (300, 570))
        
        axis_z_text = readout_font.render(f"AXIS Z: {self.axis_accelerations['Z']:.2f}g", True, readout_color)
        window.blit(axis_z_text, (300, 590))
        
        intensity_text = readout_font.render(f"INTENSITY: {math.sqrt(self.axis_accelerations['X']**2 + self.axis_accelerations['Y']**2 + self.axis_accelerations['Z']**2):.2f}g", True, readout_color)
        window.blit(intensity_text, (300, 610))
        
        # Coloana dreaptă - Informații tehnice
        samples_text = readout_font.render(f"SAMPLES: {self.sample_count:,}", True, readout_color)
        window.blit(samples_text, (600, 550))
        
        sample_rate_text = readout_font.render(f"RATE: {self.sample_rate:.0f} Hz", True, readout_color)
        window.blit(sample_rate_text, (600, 570))
        
        connected_text = readout_font.render(f"STATUS: {'CONNECTED' if self.connected else 'DISCONNECTED'}", True, readout_color)
        window.blit(connected_text, (600, 590))
        
        trigger_text = readout_font.render(f"TRIGGER: AUTO", True, readout_color)
        window.blit(trigger_text, (600, 610))
        
        # Informații despre calculul frecvenței CORECTE
        info_font = pygame.font.Font(None, 16)
        info_text = info_font.render("FFT LOGIC: Inexistent 0.5-20Hz → Moderat 8-50Hz → Puternic 20-80Hz → Devastator 45-200Hz", True, self.NAVY)
        window.blit(info_text, (25, 670))
        
        close_rect = pygame.Rect(950, 20, 40, 30)
        pygame.draw.rect(window, self.RED, close_rect)
        close_text = self.small_font.render("X", True, self.WHITE)
        window.blit(close_text, (close_rect.centerx - 5, close_rect.centery - 10))
        
        self.screen.blit(window, (100, 50))
        return close_rect.move(100, 50)
    
    def draw_duration_window(self):
        """ACTUALIZAT: Fereastra duratei cu timpul de detectare a cutremurului și text mai mare"""
        window = pygame.Surface((650, 550))  # Făcut puțin mai lat pentru textul mai mare
        # Culoarea de fundal roz deschis conform cererii (roz deschis)
        window.fill(self.LIGHT_PINK)
        
        title_font = pygame.font.Font(None, 36)  # Mărit de la 32 la 36
        title_font.set_bold(True)
        title = title_font.render("DURATA ȘI VITEZA SEISMICĂ", True, self.NAVY)
        title_rect = title.get_rect(center=(325, 30))  # Ajustat pentru lățimea nouă
        window.blit(title, title_rect)
        
        content_rect = pygame.Rect(25, 70, 600, 440)  # Mărit pentru textul mai mare
        pygame.draw.rect(window, self.WHITE, content_rect)
        pygame.draw.rect(window, self.NAVY, content_rect, 3)
        
        # Durata - cât timp este deschisă interfața
        duration = time.time() - self.start_time
        
        # Calculează timpul când s-a detectat cutremurul (timpul curent formatat)
        current_time = time.strftime("%H:%M:%S", time.localtime())
        detection_date = time.strftime("%d.%m.%Y", time.localtime())
        
        # Folosește timpul de detectare a cutremurului stocat dacă este disponibil
        if self.last_earthquake_time:
            earthquake_time_display = self.last_earthquake_time
        else:
            earthquake_time_display = current_time if self.earthquake_level != "CUTREMUR INEXISTENT" else "Nu există"
        
        if self.connected:
            values = [
                ("Durata:", f"{duration:.2f} secunde", self.BLUE),
                ("Viteza undelor seismice:", f"{self.current_velocity:.2f} m/s", self.RED),
                ("Timpul detectării cutremurului:", f"{earthquake_time_display}", self.DARK_RED),
                ("Data detectării:", f"{detection_date}", self.DARK_RED),
                ("Cât de tare se mișcă undele:", 
                 "FOARTE PUTERNIC" if self.earthquake_level == "CUTREMUR DEVASTATOR" else
                 "PUTERNIC" if self.earthquake_level == "CUTREMUR PUTERNIC" else
                 "MODERAT" if self.earthquake_level == "CUTREMUR MODERAT" else "SLAB", 
                 self.RED if self.earthquake_level in ["CUTREMUR DEVASTATOR", "CUTREMUR PUTERNIC"] else 
                 self.BRIGHT_ORANGE if self.earthquake_level == "CUTREMUR MODERAT" else self.GREEN),
                ("Integrala accelerațiilor:", f"{abs(self.velocity_integral):.2f}", self.NAVY),
                ("Media accelerație X:", f"{self.axis_accelerations['X']:.2f}g", self.RED),
                ("Media accelerație Y:", f"{self.axis_accelerations['Y']:.2f}g", self.GREEN),
                ("Media accelerație Z:", f"{self.axis_accelerations['Z']:.2f}g", self.BLUE),
                ("Samples ADC procesate:", f"{self.sample_count:,}", self.NAVY),
                ("Status detectare:", 
                 f"{self.earthquake_level}" if self.earthquake_detected else "MONITORIZARE", 
                 self.RED if self.earthquake_detected else self.DARK_GREEN)
            ]
        else:
            values = [
                ("Status sistem:", "DECONECTAT", self.RED),
                ("Durata interfață:", f"{duration:.2f} secunde", self.BLUE),
                ("Viteza undelor:", "Indisponibilă", self.GRAY),
                ("Timpul detectării:", "Nu există", self.GRAY),
                ("Pentru pornire:", "Conectați la USB și apăsați Connect", self.NAVY)
            ]
        
        y_pos = 95
        for label, value, color in values:
            if y_pos > content_rect.bottom - 60:  # Ajustat pentru spațiul mărit
                break
            
            # Font etichetă - mărit considerabil pentru citibilitate
            label_font = pygame.font.Font(None, 24)  # Mărit de la 20 la 24
            label_font.set_bold(True)
            label_text = label_font.render(label, True, self.NAVY)
            label_rect = label_text.get_rect(center=(325, y_pos))  # Centrat în noua lățime
            window.blit(label_text, label_rect)
            
            # Font valoare - mărit și mai mult pentru a fi foarte vizibil
            value_font = pygame.font.Font(None, 28)  # Mărit de la 24 la 28
            value_font.set_bold(True)
            value_text = value_font.render(str(value), True, color)
            value_rect = value_text.get_rect(center=(325, y_pos + 22))  # Ajustat spacing-ul
            window.blit(value_text, value_rect)
            
            y_pos += 38  # Mărit spacing-ul între elemente pentru textul mai mare
        
        close_rect = pygame.Rect(600, 20, 40, 30)  # Ajustat poziția pentru lățimea nouă
        pygame.draw.rect(window, self.RED, close_rect)
        close_text = self.small_font.render("X", True, self.WHITE)
        window.blit(close_text, (close_rect.centerx - 5, close_rect.centery - 10))
        
        self.screen.blit(window, (300, 150))
        return close_rect.move(300, 150)
    
    def draw_adc_window(self):
        window = pygame.Surface((1100, 650))
        window.fill(self.CREAM)  # Fundal cremă conform cererii
        
        title_font = pygame.font.Font(None, 36)
        title_font.set_bold(True)
        title = title_font.render("ADC și SPI Engine", True, self.NAVY)
        title_rect = title.get_rect(center=(550, 30))
        window.blit(title, title_rect)
        
        # Partea stângă - Starea sistemului și conversiile
        left_rect = pygame.Rect(25, 70, 520, 550)
        pygame.draw.rect(window, self.WHITE, left_rect)
        pygame.draw.rect(window, self.NAVY, left_rect, 3)
        
        status_title_font = pygame.font.Font(None, 24)
        status_title_font.set_bold(True)
        status_title = status_title_font.render("STATUS SISTEM ȘI CONVERSII", True, self.NAVY)
        window.blit(status_title, (45, 85))
        
        status_font = pygame.font.Font(None, 18)
        status_font.set_bold(True)
        
        # Starea sistemului
        status_lines = [
            f"SPI Engine: {'INIȚIALIZAT' if self.spi_engine_initialized else 'NU ESTE INIȚIALIZAT'}",
            f"AXI PWM Custom: {'INIȚIALIZAT' if self.axi_pwm_custom_initialized else 'NU ESTE INIȚIALIZAT'}",
            f"Clock Generator: {'INIȚIALIZAT' if self.clk_generator_initialized else 'NU ESTE INIȚIALIZAT'}",
            f"Rezoluția ADC: 12-bit (0-4095) → 0-8g",
            f"Conversie: 0 ADC = 0g, 4095 ADC = 8g",
            f"Factor conversie: {self.adc_to_g_factor:.6f}",
            f"Samples totale procesate: {self.sample_count:,}",
            "",
            "CONVERSII ACCELERAȚIE:",
            f"Acceleratia pe canalul 0: {self.channel_accelerations[1]:.2f}g",
            f"Acceleratia pe canalul 1: {self.channel_accelerations[1]:.2f}g", 
            f"Acceleratia pe canalul 2: {self.channel_accelerations[3]:.2f}g",
            f"Acceleratia pe canalul 3: {self.channel_accelerations[2]:.2f}g",
            f"Acceleratia pe canalul 4: {self.channel_accelerations[5]:.2f}g",
            f"Acceleratia pe canalul 5: {self.channel_accelerations[4]:.2f}g",
            "",
            "MEDII PE AXE:",
            f"Media pe X (canale 0+1): {self.axis_accelerations['X']:.2f}g",
            f"Media pe Y (canale 2+3): {self.axis_accelerations['Y']:.2f}g", 
            f"Media pe Z (canale 4+5): {self.axis_accelerations['Z']:.2f}g",
            f"SUMA celor 3 medii: {self.axis_sum:.2f}g",
            f"Media combinată FFT: {self.combined_average:.2f}g",
            "",
            f"MAGNITUDINE CURENTĂ: {self.current_magnitude:.3f}"
        ]
        
        y_offset = 110
        for line in status_lines:
            if y_offset > left_rect.bottom - 20:
                break
                
            if line == "":
                y_offset += 10
                continue
                
            if "INIȚIALIZAT" in line and not "NU ESTE" in line:
                color = self.GREEN
            elif "NU ESTE INIȚIALIZAT" in line:
                color = self.RED
            elif "Canalul" in line or "canalul" in line:
                color = self.BLUE
            elif "Media pe" in line:
                color = self.DARK_GREEN
            elif "MAGNITUDINE" in line:
                color = self.BRIGHT_RED if self.current_magnitude >= 3.0 else self.GREEN
            else:
                color = self.NAVY
            
            text = status_font.render(line, True, color)
            window.blit(text, (45, y_offset))
            y_offset += 20
        
        # Partea dreaptă - Terminal Serial Vitis
        right_rect = pygame.Rect(565, 70, 510, 550)
        pygame.draw.rect(window, (255, 250, 240), right_rect)
        pygame.draw.rect(window, self.NAVY, right_rect, 3)
        
        terminal_title_font = pygame.font.Font(None, 24)
        terminal_title_font.set_bold(True)
        terminal_title = terminal_title_font.render("VITIS SERIAL TERMINAL", True, self.NAVY)
        window.blit(terminal_title, (585, 85))
        
        # Ieșirea serială Vitis
        y_offset = 110
        max_terminal_y = right_rect.bottom - 15
        recent_lines = list(self.terminal_lines)[-28:]
        
        for line in recent_lines:
            if line.strip() and y_offset < max_terminal_y:
                color = self.NAVY
                if "Error" in line:
                    color = self.RED
                elif "SUCCES!" in line:
                    color = self.GREEN
                elif "ADC0 sample:" in line or "ADC1 sample:" in line:
                    color = self.BLUE
                elif "Starting" in line:
                    color = (128, 0, 128)
                
                display_line = line[:65] if len(line) > 65 else line
                text = self.terminal_font.render(display_line, True, color)
                window.blit(text, (585, y_offset))
                y_offset += 18
        
        close_rect = pygame.Rect(1050, 20, 40, 30)
        pygame.draw.rect(window, self.RED, close_rect)
        close_text = self.small_font.render("X", True, self.WHITE)
        window.blit(close_text, (close_rect.centerx - 5, close_rect.centery - 10))
        
        self.screen.blit(window, (50, 75))
        return close_rect.move(50, 75)
    
    def handle_click(self, pos):
        if self.boxes['acceleration'].collidepoint(pos):
            self.windows['acceleration'] = True
        elif self.boxes['richter'].collidepoint(pos):
            self.windows['magnitude'] = True
        elif self.boxes['frequency'].collidepoint(pos):
            self.windows['frequency'] = True
        elif self.boxes['duration'].collidepoint(pos):
            self.windows['duration'] = True
        elif self.boxes['adc'].collidepoint(pos):
            self.windows['adc'] = True
        elif self.connect_button_rect.collidepoint(pos):
            if self.connected:
                self.stop_serial_connection()
            else:
                self.start_serial_connection()
        elif self.port_input_rect.collidepoint(pos):
            self.input_active = True
        elif self.fullscreen_button_rect.collidepoint(pos):
            self.toggle_fullscreen()
        else:
            self.input_active = False
    
    def run(self):
        running = True
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    window_closed = False
                    
                    if self.windows['acceleration']:
                        close_rect = self.draw_acceleration_window()
                        if close_rect.collidepoint(event.pos):
                            self.windows['acceleration'] = False
                            window_closed = True
                    
                    if self.windows['magnitude'] and not window_closed:
                        close_rect = self.draw_magnitude_window()
                        if close_rect.collidepoint(event.pos):
                            self.windows['magnitude'] = False
                            window_closed = True
                    
                    if self.windows['frequency'] and not window_closed:
                        close_rect = self.draw_frequency_window()
                        if close_rect.collidepoint(event.pos):
                            self.windows['frequency'] = False
                            window_closed = True
                    
                    if self.windows['duration'] and not window_closed:
                        close_rect = self.draw_duration_window()
                        if close_rect.collidepoint(event.pos):
                            self.windows['duration'] = False
                            window_closed = True
                    
                    if self.windows['adc'] and not window_closed:
                        close_rect = self.draw_adc_window()
                        if close_rect.collidepoint(event.pos):
                            self.windows['adc'] = False
                            window_closed = True
                    
                    if not window_closed:
                        self.handle_click(event.pos)
                
                elif event.type == pygame.KEYDOWN and self.input_active:
                    if event.key == pygame.K_BACKSPACE:
                        self.serial_port_input = self.serial_port_input[:-1]
                    elif len(event.unicode) > 0 and event.unicode.isprintable():
                        self.serial_port_input += event.unicode
                
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE and self.fullscreen_mode:
                        self.toggle_fullscreen()
                    elif event.key == pygame.K_F11:
                        self.toggle_fullscreen()
            
            # Curăță ecranul
            self.screen.fill(self.BACKGROUND_COLOR)
            
            # Desenează titlul frumos
            self.draw_beautiful_title()
            
            # Desenează undele seismice animate pe părți DOAR când este conectat la Vitis
            if self.connected:
                self.draw_seismic_waves()
            
            # Desenează butonul de ecran complet
            self.draw_fullscreen_button()
            
            # Desenează controalele seriale
            pygame.draw.rect(self.screen, self.WHITE, self.port_input_rect)
            border_color = self.BLUE if self.input_active else self.NAVY
            pygame.draw.rect(self.screen, border_color, self.port_input_rect, 2)
            
            port_text = self.small_font.render(self.serial_port_input, True, self.NAVY)
            self.screen.blit(port_text, (self.port_input_rect.x + 5, self.port_input_rect.y + 5))
            
            # Butonul de conectare
            btn_color = self.RED if self.connected else self.GREEN
            btn_text = "Disconnect" if self.connected else "Connect"
            pygame.draw.rect(self.screen, btn_color, self.connect_button_rect)
            
            connect_font = pygame.font.Font(None, 20)
            connect_font.set_bold(True)
            connect_text = connect_font.render(btn_text, True, self.WHITE)
            connect_rect = connect_text.get_rect(center=self.connect_button_rect.center)
            self.screen.blit(connect_text, connect_rect)
            
            # Afișaj simplu al stării - doar tipul cutremurului pe baza combinației ADC0+ADC1
            if self.connected:
                # Stare simplă care arată doar tipul cutremurului
                status = f"CONECTAT - {self.earthquake_level}"
                
                # Culoare pe baza nivelului cutremurului din combinația ADC0+ADC1
                if self.earthquake_level == "CUTREMUR DEVASTATOR":
                    status_color = self.DARK_RED
                elif self.earthquake_level == "CUTREMUR PUTERNIC":
                    status_color = self.BRIGHT_RED
                elif self.earthquake_level == "CUTREMUR MODERAT":
                    status_color = self.BRIGHT_ORANGE
                elif self.earthquake_level == "CUTREMUR INEXISTENT":
                    status_color = self.GREEN
                else:
                    status_color = self.GREEN
            else:
                status = "DECONECTAT"
                status_color = self.RED
            
            status_font = pygame.font.Font(None, 24)
            status_font.set_bold(True)
            status_text = status_font.render(status, True, status_color)
            status_rect = status_text.get_rect(center=(self.WINDOW_WIDTH // 2, 140))
            self.screen.blit(status_text, status_rect)
            
            # Desenează casetele principale cu valori în timp real din combinația ADC0+ADC1
            # Calculează accelerația totală din mediile axelor
            total_acceleration = math.sqrt(self.axis_accelerations['X']**2 + 
                                         self.axis_accelerations['Y']**2 + 
                                         self.axis_accelerations['Z']**2)
            
            box_configs = [
                ('acceleration', "\nACCELERAȚIA", f"{total_acceleration:.2f}g", True, self.LIGHT_GREEN),
                ('richter', "\nSCARA RICHTER", f"{self.current_magnitude:.2f}", True, self.RED_PURPLE),
                ('frequency', "\nFRECVENȚA FFT", f"{self.current_frequency:.2f} Hz", True, self.ORANGE),
                ('duration', "\nDURATA ȘI VITEZA", f"{self.current_velocity:.2f} m/s", True, self.LIGHT_YELLOW),
                ('adc', "\nADC și SPI ENGINE", f"Samples: {self.sample_count}", True, self.CREAM)
            ]
            
            for box_key, title, content, clickable, color in box_configs:
                self.draw_box(self.boxes[box_key], title, content, clickable, color)
            
            # Alertă simplă de cutremur - arată doar tipul cutremurului
            if True:  # Arată întotdeauna starea cutremurului
                alert_font = pygame.font.Font(None, 48)
                alert_font.set_bold(True)
                
                # Alertă simplă care arată doar tipul cutremurului din combinația ADC0+ADC1
                if self.earthquake_level == "CUTREMUR DEVASTATOR":
                    alert_color = self.DARK_RED
                    bg_color = (255, 200, 200)
                    alert_text = alert_font.render("CUTREMUR DEVASTATOR!!", True, alert_color)
                elif self.earthquake_level == "CUTREMUR PUTERNIC":
                    alert_color = self.BRIGHT_RED
                    bg_color = (255, 220, 180)
                    alert_text = alert_font.render("CUTREMUR PUTERNIC!!", True, alert_color)
                elif self.earthquake_level == "CUTREMUR MODERAT":
                    alert_color = self.BRIGHT_ORANGE
                    bg_color = (255, 255, 200)
                    alert_text = alert_font.render("CUTREMUR MODERAT!!", True, alert_color)
                elif self.earthquake_level == "CUTREMUR INEXISTENT":
                    alert_color = self.GREEN
                    bg_color = (200, 255, 200)
                    alert_text = alert_font.render("CUTREMUR INEXISTENT", True, alert_color)
                else:
                    alert_color = self.GREEN
                    bg_color = (200, 255, 200)
                    alert_text = alert_font.render("CUTREMUR INEXISTENT", True, alert_color)
                
                # Centrează mesajul de stare al cutremurului
                alert_rect = alert_text.get_rect(center=(self.WINDOW_WIDTH // 2, self.WINDOW_HEIGHT - 60))
                
                bg_rect = pygame.Rect(alert_rect.x - 20, alert_rect.y - 10, 
                                    alert_rect.width + 40, alert_rect.height + 20)
                pygame.draw.rect(self.screen, bg_color, bg_rect)
                pygame.draw.rect(self.screen, alert_color, bg_rect, 3)
                
                self.screen.blit(alert_text, alert_rect)
            
            # Desenează ferestrele deschise
            if self.windows['acceleration']:
                self.draw_acceleration_window()
            if self.windows['magnitude']:
                self.draw_magnitude_window()
            if self.windows['frequency']:
                self.draw_frequency_window()
            if self.windows['duration']:
                self.draw_duration_window()
            if self.windows['adc']:
                self.draw_adc_window()
            
            pygame.display.flip()
            self.clock.tick(60)
        
        # Curățare
        self.stop_serial_connection()
        pygame.quit()
        sys.exit()

def main():
    try:
        monitor = SeismicMonitor()
        monitor.run()
    except Exception as e:
        print(f"Eroare monitor seismic: {e}")
        pygame.quit()
        sys.exit(1)

if __name__ == "__main__":
    main()
