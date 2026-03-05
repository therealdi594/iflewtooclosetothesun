import pygame
import random
import math

# --- Configuration ---
WIDTH, HEIGHT = 900, 700
FPS = 60
GRAVITY = 0.5
FLOOR_BOUNCE = 0.85    # High bounce for the ground
WALL_BOUNCE = 0.25     # Low bounce for slopes/walls
FRICTION_SURFACE = 0.99 
ANGULAR_DAMPING = 0.96

BG_COLOR = (15, 15, 25)
WALL_COLOR = (200, 200, 220)
HOVER_COLOR = (255, 50, 50) 
COLORS = [(0, 200, 255), (255, 100, 0), (150, 255, 50), (255, 80, 150), (200, 100, 255), (255, 215, 0)]

class PhysicsObject:
    def __init__(self, x, y, size, mass, color, shape="square"):
        self.x, self.y = x, y
        self.size = size
        self.mass = mass
        self.color = color
        self.shape = shape
        self.vx, self.vy = 0, 0
        self.angle = 0
        self.va = 0 
        self.is_dragging = False

    def get_vertices(self):
        pts = []
        sides_map = {"triangle": 3, "square": 4, "pentagon": 5, "hexagon": 6, "octagon": 8, "circle": 16}
        sides = sides_map.get(self.shape, 4)
        offset = self.angle
        if self.shape == "square": offset += math.pi/4
        elif self.shape == "triangle": offset -= math.pi/2
        
        for i in range(sides):
            ang = offset + (2 * math.pi / sides) * i
            scale = 1.3 if self.shape == "triangle" else 1.0
            px = self.x + math.cos(ang) * (self.size * scale)
            py = self.y + math.sin(ang) * (self.size * scale)
            pts.append(pygame.Vector2(px, py))
        return pts

    def update(self):
        if self.is_dragging:
            mx, my = pygame.mouse.get_pos()
            self.vx = (mx - self.x) * 0.4
            self.vy = (my - self.y) * 0.4
            self.x, self.y = mx, my
        else:
            self.vy += GRAVITY
            self.x += self.vx
            self.y += self.vy
            self.angle += self.va
            self.va *= ANGULAR_DAMPING

        # FLOOR COLLISION (High Bounce)
        verts = self.get_vertices()
        for v in verts:
            if v.y > HEIGHT:
                overlap = v.y - HEIGHT
                self.y -= overlap
                if self.vy > 0:
                    self.vy *= -FLOOR_BOUNCE
                    # Add some rotation based on horizontal speed on impact
                    self.va += self.vx * 0.01 
                break # Only need to bounce once per frame

    def draw(self, surface):
        verts = self.get_vertices()
        pygame.draw.polygon(surface, self.color, verts)
        pygame.draw.polygon(surface, (255, 255, 255), verts, 2)

def solve_collision(obj1, obj2):
    v1, v2 = obj1.get_vertices(), obj2.get_vertices()
    axes = []
    for verts in [v1, v2]:
        for i in range(len(verts)):
            p1, p2 = verts[i], verts[(i + 1) % len(verts)]
            edge = p2 - p1
            if edge.length() > 0:
                axes.append(pygame.Vector2(-edge.y, edge.x).normalize())
    
    min_overlap = 999999
    smallest_axis = None
    for axis in axes:
        dots1 = [v.dot(axis) for v in v1]
        dots2 = [v.dot(axis) for v in v2]
        if max(dots1) < min(dots2) or max(dots2) < min(dots1): return
        overlap = min(max(dots1), max(dots2)) - max(min(dots1), min(dots2))
        if overlap < min_overlap:
            min_overlap, smallest_axis = overlap, axis
            
    if smallest_axis:
        d = pygame.Vector2(obj1.x - obj2.x, obj1.y - obj2.y)
        if d.dot(smallest_axis) < 0: smallest_axis *= -1
        obj1.x += smallest_axis.x * min_overlap * 0.5
        obj1.y += smallest_axis.y * min_overlap * 0.5
        obj2.x -= smallest_axis.x * min_overlap * 0.5
        obj2.y -= smallest_axis.y * min_overlap * 0.5
        
        normal = smallest_axis
        relative_vel = pygame.Vector2(obj1.vx - obj2.vx, obj1.vy - obj2.vy)
        vel_along_normal = relative_vel.dot(normal)
        if vel_along_normal < 0:
            # Objects bouncing off each other (Medium bounce)
            mid_bounce = (FLOOR_BOUNCE + WALL_BOUNCE) / 2
            impulse = (-(1 + mid_bounce) * vel_along_normal) / 2
            obj1.vx += impulse * normal.x
            obj1.vy += impulse * normal.y
            obj2.vx -= impulse * normal.x
            obj2.vy -= impulse * normal.y

def get_closest_wall(m_pos, walls):
    closest_idx = -1
    min_dist = 6 
    for i, (p1, p2) in enumerate(walls):
        line_vec = p2 - p1
        if line_vec.length() == 0: continue
        t = max(0, min(1, (pygame.Vector2(m_pos) - p1).dot(line_vec) / line_vec.length_squared()))
        closest_point = p1 + t * line_vec
        dist = (pygame.Vector2(m_pos) - closest_point).length()
        if dist < min_dist:
            min_dist = dist
            closest_idx = i
    return closest_idx

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Consolas", 16)

    objects, walls = [], []
    drag_obj, w_start = None, None

    while True:
        m_pos = pygame.mouse.get_pos()
        screen.fill(BG_COLOR)
        hover_wall_idx = get_closest_wall(m_pos, walls)

        for event in pygame.event.get():
            if event.type == pygame.QUIT: return
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    hit_shape = False
                    for o in reversed(objects):
                        if (pygame.Vector2(o.x, o.y) - pygame.Vector2(m_pos)).length() < o.size:
                            o.is_dragging = True; drag_obj = o; hit_shape = True; break
                    if not hit_shape and hover_wall_idx != -1:
                        walls.pop(hover_wall_idx)
                elif event.button == 3: 
                    w_start = m_pos
            
            if event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1 and drag_obj:
                    drag_obj.is_dragging = False; drag_obj = None
                elif event.button == 3 and w_start:
                    walls.append((pygame.Vector2(w_start), pygame.Vector2(m_pos))); w_start = None
            
            if event.type == pygame.KEYDOWN:
                shape_map = {pygame.K_1:"circle", pygame.K_2:"square", pygame.K_3:"triangle", pygame.K_4:"pentagon", pygame.K_5:"hexagon", pygame.K_6:"octagon"}
                if event.key in shape_map:
                    objects.append(PhysicsObject(m_pos[0], m_pos[1], 25, 1, random.choice(COLORS), shape_map[event.key]))
                if event.key == pygame.K_c: objects.clear(); walls.clear()

        # Update
        for i, o1 in enumerate(objects):
            o1.update()
            for o2 in objects[i+1:]: solve_collision(o1, o2)
            
            # WALL COLLISION (Low Bounce)
            for p1, p2 in walls:
                line = p2 - p1
                if line.length() == 0: continue
                t = max(0, min(1, (pygame.Vector2(o1.x, o1.y) - p1).dot(line) / line.length_squared()))
                closest = p1 + t * line
                dist_vec = pygame.Vector2(o1.x, o1.y) - closest
                if dist_vec.length() < o1.size:
                    normal = dist_vec.normalize() if dist_vec.length() > 0 else pygame.Vector2(0, -1)
                    o1.x += normal.x * (o1.size - dist_vec.length())
                    o1.y += normal.y * (o1.size - dist_vec.length())
                    vel_dot_n = pygame.Vector2(o1.vx, o1.vy).dot(normal)
                    if vel_dot_n < 0:
                        # Apply specific Wall Bounce here
                        o1.vx -= (1 + WALL_BOUNCE) * vel_dot_n * normal.x
                        o1.vy -= (1 + WALL_BOUNCE) * vel_dot_n * normal.y
                        o1.vx *= FRICTION_SURFACE; o1.vy *= FRICTION_SURFACE

        # Draw
        for i, (p1, p2) in enumerate(walls):
            color = HOVER_COLOR if i == hover_wall_idx else WALL_COLOR
            pygame.draw.line(screen, color, p1, p2, 5)
        if w_start: pygame.draw.line(screen, (255, 255, 0), w_start, m_pos, 2)
        for o in objects: o.draw(screen)
        
        ui = ["1-6: Shapes", "Floor: Super Bouncy", "Walls: Low Bounce", "L-Click Wall: Delete"]
        for i, text in enumerate(ui):
            screen.blit(font.render(text, True, (200, 200, 200)), (20, 20 + i * 20))

        pygame.display.flip()
        clock.tick(FPS)

if __name__ == "__main__": main()
