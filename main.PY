import flet as ft
import cv2
import numpy as np
from PIL import Image
import io
import base64  # <-- Add this import
import serial
import serial.tools.list_ports
import threading
import time
import queue

def main(page: ft.Page):
    page.title = "Preparador de Imagen para Grabado Láser"
    page.vertical_alignment = ft.CrossAxisAlignment.START
    page.horizontal_alignment = ft.CrossAxisAlignment.CENTER
    page.window_width = 1920
    page.window_height = 1080

    # REFERENCIAS EXISTENTES
    image_path = ft.Ref[ft.Text]()
    input_image_display = ft.Ref[ft.Image]()
    output_image_display = ft.Ref[ft.Image]()
    threshold_slider = ft.Ref[ft.Slider]()
    canny_lower_slider = ft.Ref[ft.Slider]()
    canny_upper_slider = ft.Ref[ft.Slider]()
    blur_slider = ft.Ref[ft.Slider]()  # Nuevo slider para suavizado

    # NUEVAS REFERENCIAS PARA LA PESTAÑA DE UBICACIÓN Y ESCALA
    workspace_x_slider = ft.Ref[ft.Slider]()
    workspace_y_slider = ft.Ref[ft.Slider]()
    workspace_scale_slider = ft.Ref[ft.Slider]()
    workspace_image = ft.Ref[ft.Image]()

    # --- NUEVAS REFERENCIAS PARA GCODE ---
    serial_port_dropdown = ft.Ref[ft.Dropdown]()
    serial_status_text = ft.Ref[ft.Text]()
    gcode_textfield = ft.Ref[ft.TextField]()
    pwm_slider = ft.Ref[ft.Slider]()

    # --- NUEVAS REFERENCIAS PARA TELEMETRÍA ---
    j1_text = ft.Ref[ft.Text]()
    j2_text = ft.Ref[ft.Text]()
    x_text = ft.Ref[ft.Text]()
    y_text = ft.Ref[ft.Text]()

    # Tamaño carta en pixeles (8.5 x 11 pulgadas a 96 dpi)
    LETTER_WIDTH = 816
    LETTER_HEIGHT = 1056

    # --- NUEVOS LIMITES EN MM PARA SLIDERS ---
    WORKSPACE_X_MAX_MM = 218.9
    WORKSPACE_Y_MAX_MM = 270.4

    # Estado para la imagen procesada y parámetros de workspace
    original_image_bytes = None
    processed_image_bytes = None
    workspace_x = 0
    workspace_y = 0
    workspace_scale = 1.0

    # Variables para drag
    dragging = False
    drag_start_x = 0
    drag_start_y = 0
    img_start_x = 0
    img_start_y = 0

    # Serial variables
    serial_thread = None
    serial_obj = None
    serial_connected = False
    stop_serial_thread = False

    serial_console = ft.Ref[ft.TextField]()
    serial_connect_btn = ft.Ref[ft.ElevatedButton]()
    serial_command_input = ft.Ref[ft.TextField]()

    # --- NUEVO: Cola para gestionar el envío automático de bloques XYH ---
    xyh_send_queue = queue.Queue()
    xyh_auto_send_thread = None
    # Cambia a 'nonlocal' para uso correcto en función interna
    xyh_auto_send_active = False

    # --- NUEVO: Bandera para indicar que el hilo de envío espera XYH_MORE ---
    xyh_waiting_more = False

    def process_image():
        nonlocal original_image_bytes, processed_image_bytes

        if original_image_bytes is None:
            return

        # Convertir bytes a imagen OpenCV
        np_array = np.frombuffer(original_image_bytes, np.uint8)
        img = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

        if img is None:
            print("Error: No se pudo decodificar la imagen.")
            return

        # 1. Convertir a blanco y negro (escala de grises)
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # --- Suavizado antes de detectar bordes ---
        blur_ksize = int(blur_slider.current.value)
        if blur_ksize % 2 == 0:
            blur_ksize += 1  # El kernel debe ser impar
        if blur_ksize > 1:
            gray_img = cv2.GaussianBlur(gray_img, (blur_ksize, blur_ksize), 0)

        # Aplicar umbralización (para el "blanco y negro" puro)
        # Puedes experimentar con diferentes tipos de umbral. BINARY es bueno para láser.
        _, binary_img = cv2.threshold(gray_img, int(threshold_slider.current.value), 255, cv2.THRESH_BINARY)

        # 2. Detección de bordes con Canny
        # Los valores del slider Canny controlan la sensibilidad
        edges = cv2.Canny(gray_img, int(canny_lower_slider.current.value), int(canny_upper_slider.current.value))

        # Opcional: Invertir los bordes si deseas un "grabado de línea blanca"
        # edges = cv2.bitwise_not(edges)

        # Para simular grabado láser, los bordes son las líneas a grabar.
        # Las áreas sin bordes serán transparentes o blancas, dependiendo del formato.
        # Aquí, vamos a mostrar los bordes en negro sobre un fondo blanco para mejor visualización.
        # Creamos una imagen blanca del mismo tamaño y dibujamos los bordes negros.
        laser_sim_image = np.full(edges.shape, 255, dtype=np.uint8) # Fondo blanco
        laser_sim_image[edges > 0] = 0 # Bordes negros

        # Convertir la imagen procesada a bytes para Flet
        _, buffer = cv2.imencode('.png', laser_sim_image)
        processed_image_bytes = buffer.tobytes()

        # Convert bytes to base64 string for Flet Image
        processed_base64 = base64.b64encode(processed_image_bytes).decode("utf-8")
        output_image_display.current.src_base64 = processed_base64
        page.update()

    def update_workspace_image():
        # Muestra la imagen procesada en la hoja tamaño carta, con posición y escala
        if processed_image_bytes is None:
            workspace_image.current.src_base64 = None
            page.update()
            return

        # Crear fondo blanco tamaño carta
        carta = np.ones((LETTER_HEIGHT, LETTER_WIDTH), dtype=np.uint8) * 255

        # Leer la imagen procesada
        img_arr = np.frombuffer(processed_image_bytes, np.uint8)
        img = cv2.imdecode(img_arr, cv2.IMREAD_GRAYSCALE)
        if img is None:
            workspace_image.current.src_base64 = None
            page.update()
            return

        # Escalar la imagen
        scaled_w = int(img.shape[1] * workspace_scale)
        scaled_h = int(img.shape[0] * workspace_scale)
        if scaled_w < 1 or scaled_h < 1:
            workspace_image.current.src_base64 = None
            page.update()
            return
        img_scaled = cv2.resize(img, (scaled_w, scaled_h), interpolation=cv2.INTER_NEAREST)

        # Calcular posición
        x = int(workspace_x)
        y = int(workspace_y)
        # Pegar la imagen escalada en la hoja carta
        x_end = min(x + scaled_w, LETTER_WIDTH)
        y_end = min(y + scaled_h, LETTER_HEIGHT)
        img_x_end = x_end - x
        img_y_end = y_end - y
        if x < LETTER_WIDTH and y < LETTER_HEIGHT and img_x_end > 0 and img_y_end > 0:
            carta[y:y_end, x:x_end] = img_scaled[0:img_y_end, 0:img_x_end]

        # Codificar para mostrar en Flet
        _, buffer = cv2.imencode('.png', carta)
        carta_bytes = buffer.tobytes()
        carta_base64 = base64.b64encode(carta_bytes).decode("utf-8")
        workspace_image.current.src_base64 = carta_base64
        page.update()

    def on_workspace_pointer_down(e):
        nonlocal dragging, drag_start_x, drag_start_y, img_start_x, img_start_y
        dragging = True
        drag_start_x = e.local_x
        drag_start_y = e.local_y
        img_start_x = workspace_x
        img_start_y = workspace_y

    def on_workspace_pointer_move(e):
        nonlocal workspace_x, workspace_y, dragging, drag_start_x, drag_start_y, img_start_x, img_start_y
        if dragging:
            dx = e.local_x - drag_start_x
            dy = e.local_y - drag_start_y
            # Limitar el movimiento para que no salga del área
            new_x = int(img_start_x + dx * 2)  # *2 porque la imagen está a la mitad de tamaño
            new_y = int(img_start_y + dy * 2)
            # Limitar a los bordes de la hoja
            max_x = LETTER_WIDTH - 1
            max_y = LETTER_HEIGHT - 1
            workspace_x = max(0, min(new_x, max_x))
            workspace_y = max(0, min(new_y, max_y))
            update_workspace_image()

    def on_workspace_pointer_up(e):
        nonlocal dragging
        dragging = False

    def on_workspace_param_change(e):
        nonlocal workspace_x, workspace_y, workspace_scale
        # Actualiza valores desde sliders
        workspace_x = workspace_x_slider.current.value
        workspace_y = workspace_y_slider.current.value
        workspace_scale = workspace_scale_slider.current.value
        update_workspace_image()

    def pick_image_result(e: ft.FilePickerResultEvent):
        nonlocal original_image_bytes
        if e.files:
            file = e.files[0]
            image_path.current.value = file.name
            # Read the file bytes from the file path
            with open(file.path, "rb") as f:
                original_image_bytes = f.read()
        
            # Convert bytes to base64 string for Flet Image
            original_base64 = base64.b64encode(original_image_bytes).decode("utf-8")
            input_image_display.current.src_base64 = original_base64
            
            # Procesar la imagen inicial
            process_image()
            update_workspace_image()
            page.update()
        else:
            image_path.current.value = "No se seleccionó ninguna imagen."
            input_image_display.current.src_base64 = None
            output_image_display.current.src_base64 = None
            original_image_bytes = None
            processed_image_bytes = None
            page.update()

    file_picker = ft.FilePicker(on_result=pick_image_result)
    page.overlay.append(file_picker)

    # Función para listar puertos seriales
    def list_serial_ports():
        return [port.device for port in serial.tools.list_ports.comports()]

    # --- NUEVAS FUNCIONES PARA ENVIAR XYH ---
    BLOQUE_XYH = 50  # Tamaño del bloque de puntos a enviar

    puntos_xyh = []
    punto_xyh_index = 0
    enviando_xyh = False
    esperando_okbloque = False
    puntos_xyh_text = ft.Ref[ft.Text]()

    def mm_per_pixel():
        # Tamaño carta: 8.5 x 11 pulgadas = 215.9 x 279.4 mm
        # Imagen: 816 x 1056 px
        return 215.9 / 816, 279.4 / 1056  # (mm/px en X, mm/px en Y)

    def preparar_puntos_xyh():
        # Procesa la imagen y genera dos listas: lista_x, lista_y (como drawSquare)
        if processed_image_bytes is None:
            return [], []
        img_arr = np.frombuffer(processed_image_bytes, np.uint8)
        img = cv2.imdecode(img_arr, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return [], []
        scaled_w = int(img.shape[1] * workspace_scale)
        scaled_h = int(img.shape[0] * workspace_scale)
        if scaled_w < 1 or scaled_h < 1:
            return [], []
        img_scaled = cv2.resize(img, (scaled_w, scaled_h), interpolation=cv2.INTER_NEAREST)
        _, binary = cv2.threshold(img_scaled, 127, 255, cv2.THRESH_BINARY_INV)
        # --- CAMBIO: Usa CHAIN_APPROX_NONE para obtener TODOS los puntos de TODOS los contornos ---
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        mm_per_px_x, mm_per_px_y = mm_per_pixel()
        x_offset_mm = workspace_x
        y_offset_mm = workspace_y
        carta_mm_w = 215.9
        carta_mm_h = 279.4
        carta_px_w = LETTER_WIDTH
        carta_px_h = LETTER_HEIGHT
        px_size_x = carta_mm_w / carta_px_w
        px_size_y = carta_mm_h / carta_px_h
        x_offset_px = int(x_offset_mm / px_size_x)
        y_offset_px = int(y_offset_mm / px_size_y)
        lista_x = []
        lista_y = []
        # --- CAMBIO: Recorre TODOS los contornos y TODOS los puntos de cada contorno ---
        if len(contours) == 0:
            return [], []
        for contour in contours:
            for pt in contour:
                px = pt[0][0] + x_offset_px
                py = pt[0][1] + y_offset_px
                x_mm = px * px_size_x
                y_mm = py * px_size_y
                if 0 <= x_mm <= carta_mm_w and 0 <= y_mm <= carta_mm_h:
                    lista_x.append(x_mm)
                    lista_y.append(y_mm)
        # Visualización
        if puntos_xyh_text.current:
            puntos_xyh_text.current.value = "\n".join(
                [f"{i+1}: {x:.2f},{y:.2f}" for i, (x, y) in enumerate(zip(lista_x, lista_y))]
            )
            puntos_xyh_text.current.update()
        return lista_x, lista_y

    # --- NUEVO: Envío por bloques para evitar saturar el buffer del Arduino --- 
    # --- AJUSTE: Tamaño de bloque debe coincidir con el buffer del Arduino ---
    MAX_XYH_POINTS = 400  # Debe ser igual a MAX_XYH_POINTS en el Arduino

    def enviar_grabado_xyh(e=None):
        nonlocal xyh_auto_send_active, xyh_auto_send_thread, xyh_waiting_more
        if processed_image_bytes is None:
            serial_status_text.current.value = "No hay imagen procesada."
            page.update()
            return
        if not serial_connected or not serial_obj or not serial_obj.is_open:
            serial_status_text.current.value = "Conéctate primero al puerto serial."
            page.update()
            return
        lista_x, lista_y = preparar_puntos_xyh()
        if not lista_x or not lista_y or len(lista_x) != len(lista_y):
            serial_status_text.current.value = "No hay puntos para enviar."
            page.update()
            return

        total_points = len(lista_x)
        idx = 0

        # --- AJUSTE: Prepara los bloques de máximo 400 puntos ---
        xyh_send_queue.queue.clear()
        while idx < total_points:
            end_idx = min(idx + MAX_XYH_POINTS, total_points)
            bloque = [(lista_x[i], lista_y[i]) for i in range(idx, end_idx)]
            xyh_send_queue.put(bloque)
            idx = end_idx

        # --- Inicia el hilo de envío automático si no está activo ---
        if not xyh_auto_send_active:
            xyh_auto_send_active = True
            xyh_auto_send_thread = threading.Thread(target=xyh_auto_send_loop, daemon=True)
            xyh_auto_send_thread.start()

    def xyh_auto_send_loop():
        nonlocal xyh_auto_send_active, xyh_waiting_more
        try:
            serial_obj.reset_input_buffer()
            serial_obj.reset_output_buffer()
            serial_obj.write(b"XYH_START\r\n")
            serial_obj.flush()
            time.sleep(0.05)

            bloque_idx = 0
            total_bloques = xyh_send_queue.qsize()
            while not xyh_send_queue.empty():
                bloque = xyh_send_queue.get()
                for x, y in bloque:
                    cmd = f"XYH,{x:.2f},{y:.2f}\r\n"
                    serial_obj.write(cmd.encode())
                    serial_obj.flush()
                    time.sleep(0.001)
                bloque_idx += 1

                if not xyh_send_queue.empty():
                    serial_obj.write(b"XYH_END\r\n")
                    serial_obj.flush()
                    serial_status_text.current.value = f"Enviado bloque {bloque_idx}/{total_bloques}. Esperando Arduino..."
                    page.update()
                    # --- Espera a que serial_read_loop mande XYH_MORE ---
                    xyh_waiting_more = True
                    while xyh_auto_send_active and xyh_waiting_more:
                        time.sleep(0.01)
                else:
                    serial_obj.write(b"XYH_END\r\n")
                    serial_obj.flush()
                    time.sleep(0.05)
                    serial_obj.write(b"XYH_DONE\r\n")
                    serial_obj.flush()
                    serial_status_text.current.value = "Trayectoria XYH enviada correctamente."
                    page.update()
                    break
        except Exception as ex:
            serial_status_text.current.value = f"Error al enviar: {ex}"
            page.update()
        xyh_auto_send_active = False

    def serial_read_loop():
        nonlocal xyh_waiting_more
        # Solo muestra la consola serial y actualiza telemetría
        while not stop_serial_thread and serial_obj and serial_obj.is_open:
            try:
                if serial_obj.in_waiting > 0:
                    data = serial_obj.readline().decode(errors='ignore').strip()
                    if data:
                        # --- CAMBIO: Cuando Arduino pide NEED_XYH, responde y activa el flag para el hilo de envío ---
                        if "NEED_XYH" in data:
                            if xyh_auto_send_active:
                                serial_obj.write(b"XYH_MORE\r\n")
                                serial_obj.flush()
                                xyh_waiting_more = False
                        if data.startswith("DATA,"):
                            try:
                                _, j1, j2, x, y = data.split(",")
                                if j1_text.current: j1_text.current.value = f"J1: {float(j1):.2f}"
                                if j2_text.current: j2_text.current.value = f"J2: {float(j2):.2f}"
                                if x_text.current: x_text.current.value = f"X: {float(x):.2f}"
                                if y_text.current: y_text.current.value = f"Y: {float(y):.2f}"
                                if j1_text.current: j1_text.current.update()
                                if j2_text.current: j2_text.current.update()
                                if x_text.current: x_text.current.update()
                                if y_text.current: y_text.current.update()
                            except Exception:
                                pass
                        if serial_console.current:
                            serial_console.current.value += f"\n{data}"
                            serial_console.current.update()
            except Exception:
                break

    def conectar_serial(e=None):
        nonlocal serial_obj, serial_connected, serial_thread, stop_serial_thread
        port = serial_port_dropdown.current.value
        if not port:
            serial_status_text.current.value = "Selecciona un puerto serial."
            page.update()
            return
        try:
            if serial_obj and serial_obj.is_open:
                serial_obj.close()
            serial_obj = serial.Serial(port, 115200, timeout=1)
            serial_connected = True
            serial_status_text.current.value = f"Conectado a {port}"
            serial_connect_btn.current.text = "Desconectar"
            stop_serial_thread = False
            serial_thread = threading.Thread(target=serial_read_loop, daemon=True)
            serial_thread.start()
        except Exception as ex:
            serial_connected = False
            serial_status_text.current.value = f"Error al conectar: {ex}"
        page.update()

    def desconectar_serial(e=None):
        nonlocal serial_obj, serial_connected, stop_serial_thread
        stop_serial_thread = True
        if serial_obj:
            try:
                serial_obj.close()
            except:
                pass
        serial_obj = None
        serial_connected = False
        serial_status_text.current.value = "Desconectado."
        serial_connect_btn.current.text = "Conectar"
        page.update()

    def toggle_serial_connection(e=None):
        if serial_connected:
            desconectar_serial()
        else:
            conectar_serial()

    def enviar_comando_manual(e=None):
        if not serial_connected or not serial_obj:
            serial_status_text.current.value = "No hay conexión serial."
            page.update()
            return
        cmd = serial_command_input.current.value
        if not cmd:
            return
        try:
            serial_obj.write((cmd + '\n').encode())
            if serial_console.current:
                serial_console.current.value += f"\n> {cmd}"
                serial_console.current.update()
            serial_command_input.current.value = ""
        except Exception as ex:
            serial_status_text.current.value = f"Error al enviar: {ex}"
            page.update()

    # --- UI ACTUALIZADA SOLO PARA XYH ---
    tabs = ft.Tabs(
        selected_index=0,
        tabs=[
            ft.Tab(
                text="Procesamiento",
                content=ft.Column(
                    [
                        ft.Text("Selecciona una Imagen", size=20, weight=ft.FontWeight.BOLD),
                        ft.Row(
                            [
                                ft.ElevatedButton(
                                    "Seleccionar Imagen",
                                    icon=ft.Icons.UPLOAD_FILE,
                                    on_click=lambda _: file_picker.pick_files(allow_multiple=False),
                                ),
                                ft.Text(ref=image_path, value="Ninguna imagen seleccionada."),
                            ],
                            alignment=ft.MainAxisAlignment.CENTER
                        ),
                        ft.Divider(),
                        ft.Text("Parámetros de Procesamiento:", size=16, weight=ft.FontWeight.BOLD),
                        ft.Text("Umbral Blanco y Negro (para pre-procesamiento):", size=14),
                        ft.Slider(
                            ref=threshold_slider,
                            min=0,
                            max=255,
                            divisions=255,
                            value=127,
                            label="{value}",
                            on_change=lambda e: process_image()
                        ),
                        ft.Text("Canny (Bordes) - Umbral Inferior:", size=14),
                        ft.Slider(
                            ref=canny_lower_slider,
                            min=0,
                            max=255,
                            divisions=255,
                            value=50,
                            label="{value}",
                            on_change=lambda e: process_image()
                        ),
                        ft.Text("Canny (Bordes) - Umbral Superior:", size=14),
                        ft.Slider(
                            ref=canny_upper_slider,
                            min=0,
                            max=255,
                            divisions=255,
                            value=150,
                            label="{value}",
                            on_change=lambda e: process_image()
                        ),
                        ft.Text("Nivel de Suavizado (reduce detalles):", size=14),
                        ft.Slider(
                            ref=blur_slider,
                            min=1,
                            max=21,
                            divisions=10,
                            value=1,
                            label="{value}",
                            on_change=lambda e: process_image()
                        ),
                        ft.Divider(),
                        ft.Row(
                            [
                                ft.Column(
                                    [
                                        ft.Text("Imagen Original:", size=16),
                                        ft.Image(ref=input_image_display, width=300, height=300, fit=ft.ImageFit.CONTAIN, src_base64=""),
                                    ],
                                    horizontal_alignment=ft.CrossAxisAlignment.CENTER
                                ),
                                ft.Column(
                                    [
                                        ft.Text("Resultado (Simulación Láser):", size=16),
                                        ft.Image(ref=output_image_display, width=300, height=300, fit=ft.ImageFit.CONTAIN, src_base64=""),
                                    ],
                                    horizontal_alignment=ft.CrossAxisAlignment.CENTER
                                ),
                            ],
                            alignment=ft.MainAxisAlignment.SPACE_AROUND
                        ),
                        ft.Divider(),
                        ft.Text(
                            "Nota: El grabado se enviará como instrucciones XYH,x,y en mm sobre la hoja tamaño carta.",
                            size=12,
                            color=ft.Colors.GREY_600,
                            text_align=ft.TextAlign.CENTER
                        )
                    ]
                )
            ),
            ft.Tab(
                text="Espacio de trabajo y grabado",
                content=ft.Row(
                    [
                        # Columna izquierda: controles y consola
                        ft.Container(
                            content=ft.Column(
                                [
                                    ft.Text("Configuración de Espacio de Trabajo (Hoja tamaño carta)", size=18, weight=ft.FontWeight.BOLD),
                                    ft.Row([
                                        ft.Container(ft.Text(ref=j1_text, value="J1: -", size=16, color=ft.Colors.BLACK), padding=10, bgcolor=ft.Colors.GREY_200, border_radius=5),
                                        ft.Container(ft.Text(ref=j2_text, value="J2: -", size=16, color=ft.Colors.BLACK), padding=10, bgcolor=ft.Colors.GREY_200, border_radius=5),
                                        ft.Container(ft.Text(ref=x_text, value="X: -", size=16, color=ft.Colors.BLACK), padding=10, bgcolor=ft.Colors.GREY_200, border_radius=5),
                                        ft.Container(ft.Text(ref=y_text, value="Y: -", size=16, color=ft.Colors.BLACK), padding=10, bgcolor=ft.Colors.GREY_200, border_radius=5),
                                    ], alignment=ft.MainAxisAlignment.CENTER),
                                    ft.Text("Consola Serial", size=14, weight=ft.FontWeight.BOLD),
                                    ft.TextField(
                                        ref=serial_console,
                                        value="",
                                        multiline=True,
                                        min_lines=8,
                                        max_lines=16,
                                        read_only=True,
                                        width=400,
                                        height=200,
                                        border=ft.InputBorder.OUTLINE,
                                        text_style=ft.TextStyle(font_family="monospace", size=12),
                                    ),
                                    ft.Row([
                                        ft.TextField(
                                            ref=serial_command_input,
                                            label="Comando manual",
                                            width=250,
                                            on_submit=enviar_comando_manual
                                        ),
                                        ft.ElevatedButton(
                                            text="Enviar",
                                            on_click=enviar_comando_manual
                                        )
                                    ], alignment=ft.MainAxisAlignment.START),
                                    ft.Text("Ubicación X (mm):"),
                                    ft.Slider(
                                        ref=workspace_x_slider,
                                        min=0,
                                        max=WORKSPACE_X_MAX_MM,
                                        divisions=20,
                                        value=0,
                                        label="{value:.1f}".replace("{value:.1f}", "{value}"),
                                        on_change=on_workspace_param_change,
                                        width=250
                                    ),
                                    ft.Text("Ubicación Y (mm):"),
                                    ft.Slider(
                                        ref=workspace_y_slider,
                                        min=0,
                                        max=WORKSPACE_Y_MAX_MM,
                                        divisions=20,
                                        value=0,
                                        label="{value:.1f}".replace("{value:.1f}", "{value}"),
                                        on_change=on_workspace_param_change,
                                        width=250
                                    ),
                                    ft.Text("Escala de la imagen:"),
                                    ft.Slider(
                                        ref=workspace_scale_slider,
                                        min=0.1,
                                        max=2.0,
                                        divisions=19,
                                        value=1.0,
                                        label="{value:.2f}".replace("{value:.2f}", "{value}"),
                                        on_change=on_workspace_param_change,
                                        width=250
                                    ),
                                    ft.Text(ref=serial_status_text, value="", color=ft.Colors.BLUE, size=12),
                                ],
                                spacing=10,
                                alignment=ft.MainAxisAlignment.START,
                                horizontal_alignment=ft.CrossAxisAlignment.START,
                            ),
                            width=450,
                            padding=20,
                        ),
                        # Columna derecha: área de trabajo y botón
                        ft.Container(
                            content=ft.Column(
                                [
                                    ft.Container(
                                        content=ft.Image(
                                            ref=workspace_image,
                                            width=408,
                                            height=528,
                                            fit=ft.ImageFit.CONTAIN,
                                            src_base64=None
                                        ),
                                        bgcolor=ft.Colors.WHITE,
                                        border=ft.border.all(1, ft.Colors.GREY_400),
                                        width=408,
                                        height=528,
                                        alignment=ft.alignment.center,
                                    ),
                                    ft.Row([
                                        ft.ElevatedButton("Enviar grabado XYH", on_click=enviar_grabado_xyh),
                                    ], alignment=ft.MainAxisAlignment.CENTER),
                                ],
                                alignment=ft.MainAxisAlignment.START,
                                horizontal_alignment=ft.CrossAxisAlignment.CENTER,
                                spacing=20,
                            ),
                            padding=20,
                        ),
                    ],
                    alignment=ft.MainAxisAlignment.START,
                    vertical_alignment=ft.CrossAxisAlignment.START,
                )
            ),
            ft.Tab(
                text="Serial",
                content=ft.Column([
                    ft.Text("Conexión Serial", size=18, weight=ft.FontWeight.BOLD),
                    ft.Row([
                        ft.Text("Puerto:"),
                        ft.Dropdown(
                            ref=serial_port_dropdown,
                            options=[ft.dropdown.Option(p) for p in list_serial_ports()],
                            width=200,
                        ),
                        ft.IconButton(
                            icon=ft.icons.REFRESH,
                            tooltip="Actualizar puertos",
                            on_click=lambda e: (
                                serial_port_dropdown.current.options.clear(),
                                serial_port_dropdown.current.options.extend([ft.dropdown.Option(p) for p in list_serial_ports()]),
                                page.update()
                            ),
                        ),
                        ft.ElevatedButton(
                            ref=serial_connect_btn,
                            text="Conectar",
                            on_click=toggle_serial_connection,
                        ),
                    ], alignment=ft.MainAxisAlignment.START),
                    ft.Text(ref=serial_status_text, value="", color=ft.Colors.BLUE, size=12),
                    # Eliminado: consola serial y envío de comandos manuales
                ], spacing=10)
            ),
            # NUEVA PESTAÑA: Visualización de puntos XYH
            ft.Tab(
                text="Puntos XYH",
                content=ft.Container(
                    content=ft.Column(
                        [
                            ft.Text("Lista completa de puntos XYH a visitar:", size=16, weight=ft.FontWeight.BOLD),
                            ft.Text(ref=puntos_xyh_text, value="", size=12, selectable=True, max_lines=40, width=600),
                        ],
                        scroll=ft.ScrollMode.AUTO,
                        expand=True,
                    ),
                    padding=20,
                    expand=True,
                )
            ),
        ]
    )

    page.add(tabs)

    # Inicializa la vista previa al cargar la página
    def on_page_load(e):
        pass
    page.on_load = on_page_load

if __name__ == "__main__":
    ft.app(target=main)