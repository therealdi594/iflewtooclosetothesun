import pygame
import random
import math

# --- Configuration ---
WIDTH, HEIGHT = 900, 700
FPS = 60
GRAVITY = 0.45
FLOOR_BOUNCE = 0.8
WALL_BOUNCE = 0.4
FRICTION_SURFACE = 0.985
ANGULAR_DAMPING = 0.98
SUB_STEPS = 4  # Balanced sub-steps for smooth, lag-free performance

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
        self.cached_verts = []

    @property
    def inertia(self):
        return 0.5 * self.mass * (self.size ** 2)

    def update_vertices(self):
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
        self.cached_verts = pts

    def update_substep(self, dt_gravity):
        if self.is_dragging:
            mx, my = pygame.mouse.get_pos()
            self.vx = (mx - self.x) * 0.4
            self.vy = (my - self.y) * 0.4
            self.x, self.y = mx, my
            self.va = 0
        else:
            self.vy += dt_gravity
            self.x += self.vx / SUB_STEPS
            self.y += self.vy / SUB_STEPS
            self.angle += self.va / SUB_STEPS
            self.va *= (ANGULAR_DAMPING ** (1 / SUB_STEPS))

    def draw(self, surface):
        pygame.draw.polygon(surface, self.color, self.cached_verts)
        pygame.draw.polygon(surface, (255, 255, 255), self.cached_verts, 2)

def solve_screen_bounds(obj):
    if obj.x - obj.size < 0:
        obj.x = obj.size
        if obj.vx < 0: obj.vx *= -WALL_BOUNCE
    elif obj.x + obj.size > WIDTH:
        obj.x = WIDTH - obj.size
        if obj.vx > 0: obj.vx *= -WALL_BOUNCE
        
    if obj.y - obj.size < 0:
        obj.y = obj.size
        if obj.vy < 0: obj.vy *= -WALL_BOUNCE
    elif obj.y + obj.size > HEIGHT:
        obj.y = HEIGHT - obj.size
        if obj.vy > 0:
            obj.vy *= -FLOOR_BOUNCE
            obj.vx *= FRICTION_SURFACE
            obj.va += obj.vx * 0.002

def solve_collision(obj1, obj2):
    v1, v2 = obj1.cached_verts, obj2.cached_verts
    axes = []
    for verts in [v1, v2]:
        for i in range(len(verts)):
            p1, p2 = verts[i], verts[(i + 1) % len(verts)]
            edge = p2 - p1
            if edge.length() > 0:
                axes.append(pygame.Vector2(-edge.y, edge.x).normalize())
    
    min_overlap = float('inf')
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
        
        total_mass = obj1.mass + obj2.mass
        ratio1 = obj2.mass / total_mass
        ratio2 = obj1.mass / total_mass
        
        correction = min_overlap * 0.5
        obj1.x += smallest_axis.x * correction * ratio1
        obj1.y += smallest_axis.y * correction * ratio1
        obj2.x -= smallest_axis.x * correction * ratio2
        obj2.y -= smallest_axis.y * correction * ratio2
        
        normal = smallest_axis
        relative_vel = pygame.Vector2(obj1.vx - obj2.vx, obj1.vy - obj2.vy)
        vel_along_normal = relative_vel.dot(normal)
        
        if vel_along_normal < 0:
            restitution = (FLOOR_BOUNCE + WALL_BOUNCE) / 2
            impulse_scalar = -(1 + restitution) * vel_along_normal / (1/obj1.mass + 1/obj2.mass)
            
            impulse = normal * impulse_scalar
            obj1.vx += (impulse.x / obj1.mass)
            obj1.vy += (impulse.y / obj1.mass)
            obj2.vx -= (impulse.x / obj2.mass)
            obj2.vy -= (impulse.y / obj2.mass)
            
            obj1.va += (impulse.x * 0.02) / (obj1.inertia / obj1.mass)
            obj2.va -= (impulse.x * 0.02) / (obj2.inertia / obj2.mass)

def solve_wall_collision(obj, walls):
    for p1, p2 in walls:
        line_vec = p2 - p1
        line_len = line_vec.length()
        if line_len == 0: continue
        line_dir = line_vec / line_len
        wall_normal = pygame.Vector2(-line_dir.y, line_dir.x)
        
        for v in obj.cached_verts:
            v_vec = v - p1
            t = v_vec.dot(line_dir)
            if 0 <= t <= line_len:
                closest = p1 + t * line_dir
                dist_vec = v - closest
                dist = dist_vec.length()
                
                if dist < 4.0 or dist_vec.dot(wall_normal) < 0:
                    normal = wall_normal if dist_vec.dot(wall_normal) >= 0 else -wall_normal
                    if dist > 0:
                        normal = dist_vec.normalize()
                    
                    # Calculate velocity at the specific contact vertex (incorporating rotation/torque)
                    r = v - pygame.Vector2(obj.x, obj.y)
                    v_vert = pygame.Vector2(obj.vx - obj.va * r.y, obj.vy + obj.va * r.x)
                    vel_along_normal = v_vert.dot(normal)
                    
                    if vel_along_normal < 0:
                        # True impulse-momentum response with rotational torque
                        r_cross_n = r.x * normal.y - r.y * normal.x
                        inv_inertia = 1.0 / obj.inertia
                        denominator = (1.0 / obj.mass) + (r_cross_n * r_cross_n) * inv_inertia
                        
                        if denominator > 0:
                            impulse_scalar = -(1.0 + WALL_BOUNCE) * vel_along_normal / denominator
                            impulse = normal * impulse_scalar
                            
                            # Apply velocity change naturally (no teleporting/snapping)
                            obj.vx += impulse.x / obj.mass
                            obj.vy += impulse.y / obj.mass
                            
                            torque = r.x * impulse.y - r.y * impulse.x
                            obj.va += torque * inv_inertia
                            
                            # Extremely minor nudge only to prevent deep sinking
                            penetration = 3.0 - dist
                            if penetration > 0:
                                obj.x += normal.x * penetration * 0.1
                                obj.y += normal.y * penetration * 0.1

def get_closest_wall(m_pos, walls):
    closest_idx = -1
    min_dist = 8 
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
        m_pos = pygame.Vector2(pygame.mouse.get_pos())
        screen.fill(BG_COLOR)
        hover_wall_idx = get_closest_wall(m_pos, walls)

        for event in pygame.event.get():
            if event.type == pygame.QUIT: return
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    hit_shape = False
                    for o in reversed(objects):
                        if (pygame.Vector2(o.x, o.y) - m_pos).length() < o.size:
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
                    objects.append(PhysicsObject(m_pos.x, m_pos.y, 25, 1, random.choice(COLORS), shape_map[event.key]))
                elif event.key == pygame.K_e:
                    for o in reversed(objects):
                        if (pygame.Vector2(o.x, o.y) - m_pos).length() < o.size:
                            objects.remove(o)
                            break
                elif event.key == pygame.K_c: 
                    objects.clear(); walls.clear()

        # Update cached vertices once before physics loop
        for o in objects:
            o.update_vertices()

        # Physics Sub-Stepping Loop
        dt_gravity = GRAVITY / SUB_STEPS
        for _ in range(SUB_STEPS):
            for o1 in objects:
                o1.update_substep(dt_gravity)
                solve_screen_bounds(o1)
                solve_wall_collision(o1, walls)
                o1.update_vertices()

            # Broad-Phase Optimized Object-to-Object Collisions
            for i, o1 in enumerate(objects):
                for o2 in objects[i+1:]: 
                    dx = o1.x - o2.x
                    dy = o1.y - o2.y
                    if dx*dx + dy*dy < (o1.size + o2.size + 15)**2:
                        solve_collision(o1, o2)

        # Final vertex update for rendering
        for o in objects:
            o.update_vertices()

        # Draw Walls
        for i, (p1, p2) in enumerate(walls):
            color = HOVER_COLOR if i == hover_wall_idx else WALL_COLOR
            pygame.draw.line(screen, color, p1, p2, 5)
        if w_start: 
            pygame.draw.line(screen, (255, 255, 0), w_start, m_pos, 2)
            
        # Draw Objects
        for o in objects: 
            o.draw(screen)
        
        # UI Instructions
        ui = [
            "1-6: Spawn Shapes | E: Delete Shape under cursor",
            "Left Click: Drag shapes / Delete wall | Right Click: Draw Wall",
            "C: Clear All | True Impulse-Momentum Wall Collisions Active!"
        ]
        for i, text in enumerate(ui):
            screen.blit(font.render(text, True, (200, 200, 200)), (20, 20 + i * 20))

        pygame.display.flip()
        clock.tick(FPS)

if __name__ == "__main__": 
    main()