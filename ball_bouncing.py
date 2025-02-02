import pygame
import math
import sys


# Initialize Pygame
pygame.init()

# Screen parameters
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Bouncing Ball in a Spinning Hexagon")
clock = pygame.time.Clock()

# Colors
BG_COLOR = (30, 30, 30)
HEX_COLOR = (200, 200, 200)
BALL_COLOR = (255, 100, 100)

# Physics parameters
GRAVITY = 500.0             # pixels per second^2 downward
AIR_FRICTION = 0.0          # optional drag in the air (set to 0 for no drag)
RESTITUTION = 0.9           # bounce factor (0: inelastic, 1: elastic)
COLLISION_FRICTION = 0.2    # friction applied during collision (reduces tangential velocity)

# Ball parameters
ball_radius = 15
ball_pos = pygame.math.Vector2(WIDTH // 2, HEIGHT // 2 - 100)
ball_vel = pygame.math.Vector2(150, 0)

# Hexagon parameters
hex_center = pygame.math.Vector2(WIDTH // 2, HEIGHT // 2)
hex_radius = 250            # distance from center to vertex
num_sides = 6
hex_angle = 0               # current rotation angle in radians
angular_speed = math.radians(60)  # 30 degrees per second in radians

def get_hexagon_vertices(center, radius, sides, angle_offset):
    """Return the vertices of a regular polygon (hexagon) given a center, radius, number of sides, and rotation."""
    vertices = []
    for i in range(sides):
        theta = angle_offset + 2 * math.pi * i / sides
        x = center.x + radius * math.cos(theta)
        y = center.y + radius * math.sin(theta)
        vertices.append(pygame.math.Vector2(x, y))
    return vertices

def closest_point_on_segment(p, a, b):
    """Return the closest point on the line segment ab to point p."""
    ap = p - a
    ab = b - a
    t = ap.dot(ab) / (ab.length_squared() if ab.length_squared() != 0 else 1)
    t = max(0, min(1, t))
    return a + ab * t

def wall_velocity_at_point(point, center, ang_speed):
    """
    For a rotating polygon, every point rotates about the center.
    The velocity is perpendicular to the radius vector and proportional to the angular speed.
    """
    r = point - center
    # Perpendicular vector: (-r.y, r.x)
    return pygame.math.Vector2(-r.y, r.x) * ang_speed

def handle_collision(ball_pos, ball_vel, vertices, dt):
    """
    Check collision between the ball and each edge of the hexagon.
    If a collision is detected, adjust the ball's velocity using
    a reflection based on the moving wall’s velocity.
    """
    for i in range(len(vertices)):
        a = vertices[i]
        b = vertices[(i + 1) % len(vertices)]
        # Compute the closest point Q on the edge (line segment) to the ball center.
        q = closest_point_on_segment(ball_pos, a, b)
        # Vector from Q to ball center
        delta = ball_pos - q
        dist = delta.length()

        # Check for collision: if the ball is overlapping the wall.
        if dist < ball_radius:
            # Compute the normal (pointing from wall to ball)
            if dist == 0:
                # Avoid division by zero
                normal = pygame.math.Vector2(0, -1)
            else:
                normal = delta / dist

            # Compute the wall’s velocity at point q.
            wall_vel = wall_velocity_at_point(q, hex_center, angular_speed)

            # Compute ball’s velocity relative to the wall.
            rel_vel = ball_vel - wall_vel

            # Only reflect if the relative velocity is pointing into the wall.
            rel_normal_vel = rel_vel.dot(normal)
            if rel_normal_vel < 0:
                # Reflect the normal component (with restitution)
                v_n = normal * rel_normal_vel
                v_t = rel_vel - v_n  # tangential component

                # Reflect the normal component
                v_n = -RESTITUTION * v_n
                # Apply friction to the tangential component
                v_t *= (1 - COLLISION_FRICTION)

                # New relative velocity
                new_rel_vel = v_n + v_t

                # Convert back to absolute velocity
                ball_vel = new_rel_vel + wall_vel

                # Push the ball out of collision so it is exactly at the surface.
                overlap = ball_radius - dist
                ball_pos += normal * overlap

    return ball_pos, ball_vel

# Main loop
running = True
while running:
    dt = clock.tick(60) / 1000.0  # seconds elapsed since last frame

    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Update hexagon rotation
    hex_angle += angular_speed * dt
    vertices = get_hexagon_vertices(hex_center, hex_radius, num_sides, hex_angle)

    # Apply gravity to ball velocity
    ball_vel.y += GRAVITY * dt
    # Optionally apply air friction (if desired)
    ball_vel *= (1 - AIR_FRICTION * dt)

    # Update ball position
    ball_pos += ball_vel * dt

    # Handle collision with the hexagon edges
    ball_pos, ball_vel = handle_collision(ball_pos, ball_vel, vertices, dt)

    # Clear screen
    screen.fill(BG_COLOR)

    # Draw hexagon
    pygame.draw.polygon(screen, HEX_COLOR, [v for v in vertices], 3)

    # Draw ball
    pygame.draw.circle(screen, BALL_COLOR, (int(ball_pos.x), int(ball_pos.y)), ball_radius)

    # Update display
    pygame.display.flip()

pygame.quit()
sys.exit()
