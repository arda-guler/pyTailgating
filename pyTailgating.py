import pygame
import sys
from collections import deque
import math

# =============
# Configuration
# =============
WIDTH, HEIGHT = 1000, 400
FPS = 60

CAR_WIDTH, CAR_HEIGHT = 60, 30

# "aggressiveness" levels
PROFILES = {
    "im too young to die": 0.0, # ehe DoOm reFeREncE
    "new driver": 0.1,
    "kids onboard": 0.15,
    "elderly": 0.2,
    "cheap junk": 0.3,
    "still civilized": 0.4,
    "in a hurry": 0.5,
    "well ok...": 0.6,
    "nurburgring": 0.7,
    "alligator tailgator": 0.8,
    "Bayerische Motoren Werke": 0.9,
    "rushing crap to the tannery": 1.0 # idiom, don't ask
}

profile = PROFILES["well ok..."]

TARGET_DISTANCE = 150 - 5 * profile
DEADBAND = 6

MAX_SPEED = 350
MAX_ACCEL = 100
MAX_BRAKE = -300
MAX_JERK = 1000

REACTION_TIME = 0.5

STANDSTILL_DISTANCE = 25
TIME_HEADWAY = 1.8 - profile

# time-to-contact
DESIRED_TTC = 2.6 - profile
MIN_TTC = 1.5 - profile
TTC_GAIN = 600

DASH_LENGTH = 40
GAP_LENGTH = 30
ROAD_LINE_HEIGHT = 6

BRAKE_LIGHT_WIDTH = 10
BRAKE_LIGHT_HEIGHT = 6
BRAKE_LIGHT_OFFSET = 2

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

    def reset(self):
        self.integral = 0
        self.prev_error = 0

class Car:
    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.speed = 0
        self.color = color

    def update(self, accel, dt):
        self.speed += accel * dt
        self.speed = max(0, min(self.speed, MAX_SPEED))
        self.x += self.speed * dt

    def draw(self, screen, braking=False):
        car_rect = pygame.Rect(
            self.x - CAR_WIDTH // 2,
            self.y - CAR_HEIGHT // 2,
            CAR_WIDTH,
            CAR_HEIGHT,
        )

        pygame.draw.rect(screen, self.color, car_rect)

        # brake lights!
        if braking:
            left_light = pygame.Rect(
                car_rect.left + BRAKE_LIGHT_OFFSET,
                car_rect.centery - BRAKE_LIGHT_HEIGHT // 2 - 12,
                BRAKE_LIGHT_WIDTH,
                BRAKE_LIGHT_HEIGHT,
            )

            right_light = pygame.Rect(
                car_rect.left + BRAKE_LIGHT_OFFSET,
                car_rect.centery - BRAKE_LIGHT_HEIGHT // 2 + 12,
                BRAKE_LIGHT_WIDTH,
                BRAKE_LIGHT_HEIGHT,
            )

            pygame.draw.rect(screen, (255, 0, 0), left_light)
            pygame.draw.rect(screen, (255, 0, 0), right_light)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("pyTailgating")
    clock = pygame.time.Clock()

    lead_car = Car(WIDTH // 2, HEIGHT // 2, (60, 200, 60))
    follower_car = Car(WIDTH // 2 - TARGET_DISTANCE, HEIGHT // 2, (200, 60, 60))

    pid_accel = PIDController(0.6 + profile * 0.02, 0.0, 0.2)
    pid_brake = PIDController(1.2 + profile * 0.02, 0.0, 0.6)

    reaction_buffer = deque()

    road_offset = 0
    last_accel = 0

    running = True
    while running:
        dt = clock.tick(FPS) / 1000

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # user car driving
        keys = pygame.key.get_pressed()
        user_accel = 0
        if keys[pygame.K_w]:
            user_accel = MAX_ACCEL
        elif keys[pygame.K_s]:
            user_accel = MAX_BRAKE

        lead_car.update(user_accel, dt)
        
        lead_braking = user_accel < 0

        # delayed perception
        true_distance = lead_car.x - follower_car.x
        true_rel_speed = follower_car.speed - lead_car.speed
        
        reaction_buffer.append({"distance": true_distance, "rel_speed": true_rel_speed})

        max_len = int(REACTION_TIME * FPS)
        if len(reaction_buffer) > max_len:
            reaction_buffer.popleft()

        perceived = reaction_buffer[0]

        distance = perceived["distance"]
        rel_speed = perceived["rel_speed"]
        min_follow_distance = STANDSTILL_DISTANCE + follower_car.speed * TIME_HEADWAY

        if rel_speed > 0:
            ttc = distance / rel_speed
        else:
            ttc = float("inf")

        desired_accel = 0

        distance_error = distance - max(TARGET_DISTANCE, min_follow_distance)

        if abs(distance_error) < DEADBAND:
            pid_accel.reset()
            pid_brake.reset()
            desired_accel = 0

        if distance < min_follow_distance:
            pid_accel.reset()
            desired_accel = MAX_BRAKE

        elif ttc < DESIRED_TTC:
            pid_accel.reset()

            if ttc < MIN_TTC:
                desired_accel = MAX_BRAKE
            else:
                ttc_error = DESIRED_TTC - ttc
                desired_accel = -TTC_GAIN * ttc_error

                # Distance trim
                trim = pid_brake.update(distance_error, dt)
                desired_accel += trim
                desired_accel = max(desired_accel, MAX_BRAKE)

        else:
            pid_brake.reset()
            desired_accel = pid_accel.update(distance_error, dt)
            desired_accel = min(desired_accel, MAX_ACCEL)

        max_delta = MAX_JERK * dt
        desired_accel = max(last_accel - max_delta, min(last_accel + max_delta, desired_accel))
        last_accel = desired_accel

        follower_car.update(desired_accel, dt)

        follower_braking = desired_accel < -25

        # =============
        # GRAPHICS
        # =============
        world_shift = lead_car.speed * dt
        road_offset -= world_shift

        follower_car.x -= world_shift
        lead_car.x = WIDTH // 2

        screen.fill((30, 30, 30))

        pattern = DASH_LENGTH + GAP_LENGTH
        y = HEIGHT // 2
        start_x = road_offset % pattern - pattern
        x = start_x
        while x < WIDTH:
            pygame.draw.rect(screen, (220, 220, 220), (x, y + 20, DASH_LENGTH, ROAD_LINE_HEIGHT))
            pygame.draw.rect(screen, (220, 220, 220), (x, y - 30, DASH_LENGTH, ROAD_LINE_HEIGHT))
            x += pattern

        lead_car.draw(screen, braking=lead_braking)
        follower_car.draw(screen, braking=follower_braking)

        font = pygame.font.SysFont(None, 22)
        hud = font.render(
            f"Lead Speed: {int(lead_car.speed)}  "
            f"Gap: {int(true_distance)}  "
            f"TTC: {ttc:.2f}",
            True, (255, 255, 255))
        screen.blit(hud, (10, 10))

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
