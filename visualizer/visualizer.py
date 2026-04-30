import serial
import numpy as np
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

# =========================
# CONFIG
# =========================

PORT = "COM3"        # CHANGE THIS
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

# =========================
# DH PARAMETERS (MATCH ESP32)
# =========================

# (a, d, alpha)
dh = [
    (0, 131.22, np.pi/2),
    (-110.4, 0, 0),
    (-96, 0, 0),
    (0, 63.4, np.pi/2),
    (0, 75.05, -np.pi/2),
    (0, 45.6, 0)
]

# =========================
# MATH
# =========================

def dh_transform(a, d, alpha, theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])

def forward_kinematics(q):
    T = np.eye(4)
    points = [T[:3, 3].copy()]

    for i in range(len(q)):
        a, d, alpha = dh[i]
        T = T @ dh_transform(a, d, alpha, q[i])
        points.append(T[:3, 3].copy())

    return points

# =========================
# SERIAL PARSER
# =========================

def read_joints():
    try:
        line = ser.readline().decode().strip()

        if not line.startswith("Q:"):
            return None

        vals = line[2:].strip().split()

        if len(vals) < 6:
            return None

        q = np.array([float(v) for v in vals[:6]])
        return q

    except:
        return None

# =========================
# RENDERING
# =========================

def draw_robot(points):
    glColor3f(0.2, 0.8, 1.0)
    glLineWidth(3)

    glBegin(GL_LINES)
    for i in range(len(points) - 1):
        glVertex3fv(points[i])
        glVertex3fv(points[i + 1])
    glEnd()

    # draw joints as points
    glPointSize(8)
    glBegin(GL_POINTS)
    for p in points:
        glVertex3fv(p)
    glEnd()

# =========================
# OPENGL INIT
# =========================

pygame.init()
display = (900, 700)
pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

gluPerspective(45, display[0] / display[1], 0.1, 100.0)
glTranslatef(0, -0.5, -2.5)
glEnable(GL_DEPTH_TEST)

# camera rotation
rot_x = 20
rot_y = -30

# =========================
# MAIN LOOP
# =========================

q = np.zeros(6)

while True:

    # -------------------------
    # EVENTS
    # -------------------------
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

        if event.type == pygame.MOUSEMOTION:
            if pygame.mouse.get_pressed()[0]:
                rot_y += event.rel[0]
                rot_x += event.rel[1]

    # -------------------------
    # READ DATA
    # -------------------------
    new_q = read_joints()
    if new_q is not None:
        q = new_q

    # -------------------------
    # FK
    # -------------------------
    points = forward_kinematics(q)

    # -------------------------
    # RENDER
    # -------------------------
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glPushMatrix()
    glRotatef(rot_x, 1, 0, 0)
    glRotatef(rot_y, 0, 1, 0)

    draw_robot(points)

    glPopMatrix()

    pygame.display.flip()
    pygame.time.wait(10)