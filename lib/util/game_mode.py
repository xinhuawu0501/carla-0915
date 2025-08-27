import pygame
import numpy as np
import math

camera_surface = None  # global image buffer


def process_image(image):
    global camera_surface
    # Convert raw BGRA to RGB array
    img_array = np.frombuffer(image.raw_data, dtype=np.uint8)
    img_array = img_array.reshape((image.height, image.width, 4))[:, :, :3]  # Drop alpha
    img_array = img_array[:, :, ::-1]  # Convert BGR to RGB

    # Convert to pygame surface
    camera_surface = pygame.surfarray.make_surface(img_array.swapaxes(0, 1))


def process_control(vehicle, event, control):
    control.throttle = 0
    control.steer = 0
    control.brake = 0

    v_vec = vehicle.get_velocity()
    v_speed = math.fabs(math.sqrt(v_vec.x**2 + v_vec.y**2 + v_vec.z**2))
    
    is_stat = v_speed < 0.01

    done = False

    if event.type == pygame.KEYDOWN:
        if event.key == pygame.K_UP:
            print("KEYDOWN K_UP")
            control.reverse = False
            control.throttle = 1     
        elif event.key == pygame.K_DOWN:
            print("KEYDOWN K_DOWN")
            control.brake = 0.5
            if is_stat:
                control.brake = 0
                control.reverse = True
                control.throttle = 0.5
                control.gear = 1
            
        elif event.key == pygame.K_LEFT:
            print("KEYDOWN K_LEFT")
            control.steer = -1
        elif event.key == pygame.K_RIGHT:
            print("KEYDOWN K_RIGHT")
            control.steer = 1
        elif event.key == pygame.K_SPACE:
            print("quit game")
            pygame.quit()
            done = True
    #unpressed
    elif event.type == pygame.KEYUP:
        print("KEYUP")
        control.throttle = 0
        control.steer = 0
        control.brake = 0
        
    vehicle.apply_control(control)
    return done

