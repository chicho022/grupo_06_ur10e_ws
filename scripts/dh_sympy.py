#!/usr/bin/env python3
"""UR10e (Grupo 6) - Paso 3: DH estándar, T06, p(q), R(q) y comprobaciones.

Convención de frames (la del docente): x1 y x2 apuntan hacia el brazo,
x3...x6 hacia -brazo (igual que x0). Por eso hay offsets de pi en theta1 y theta3.
Todos los valores d, a vienen de physical_parameters.yaml (verifícalos).
Unidades: metros y radianes.
"""
import sympy as sp
import numpy as np

# ---------------------------------------------------------------- parámetros
D1, A2, A3 = sp.Rational(1807, 10000), sp.Rational(6127, 10000), sp.Rational(57155, 100000)
D4, D5, D6 = sp.Rational(17415, 100000), sp.Rational(11985, 100000), sp.Rational(11655, 100000)

q = sp.symbols("q1:7", real=True)           # q1..q6
pi = sp.pi

# (offset de theta, d, a, alpha)  ->  theta_i = q_i + offset
TABLA = [
    (pi,  D1,  0,   -pi / 2),   # A1: base -> hombro
    (0,   0,   A2,   0),        # A2: hombro -> codo
    (pi,  0,  -A3,   0),        # A3: codo -> muñeca 1
    (0,   D4,  0,    pi / 2),   # A4: muñeca 1 -> muñeca 2
    (0,   D5,  0,   -pi / 2),   # A5: muñeca 2 -> muñeca 3
    (0,   D6,  0,    0),        # A6: muñeca 3 -> tool0
]


def dh(theta, d, a, alpha):
    """Matriz genérica DH estándar: Rz(theta) Tz(d) Tx(a) Rx(alpha)."""
    ct, st, ca, sa = sp.cos(theta), sp.sin(theta), sp.cos(alpha), sp.sin(alpha)
    return sp.Matrix([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1],
    ])


def simp(M):
    return M.applyfunc(lambda e: sp.trigsimp(sp.nsimplify(e, rational=True)))


# ---------------------------------------------------------------- A1...A6
A = [simp(dh(q[i] + off, d, a, al)) for i, (off, d, a, al) in enumerate(TABLA)]

# ---------------------------------------------------------------- T06
T06 = sp.eye(4)
for Ai in A:
    T06 = T06 * Ai
T06 = simp(T06)
p = T06[:3, 3]
R = T06[:3, :3]

# ---------------------------------------------------------------- LaTeX
print("%% ---- LaTeX para Overleaf ----")
for i, Ai in enumerate(A, 1):
    print(f"%% A{i}\n{sp.latex(Ai)}\n")
print(f"%% p(q)\n{sp.latex(p)}\n")
print(f"%% R(q)\n{sp.latex(R)}\n")

# ---------------------------------------------------------------- caso #1
Q1 = [-4.273603522042736, -1.0610410581420684, -1.6986263087082847,
      -4.805964006705923, -1.3640204123167399, 0.44281597917836635]
P_TF = np.array([0.116, 0.219, 0.785])
QUAT_TF = np.array([0.017, 0.984, 0.173, 0.031])   # x, y, z, w

T = np.array(T06.subs(dict(zip(q, Q1))).evalf(), dtype=float)
Rn, pn = T[:3, :3], T[:3, 3]

print("== Caso #1 ==")
print("T06 =\n", np.round(T, 4))
print("p DH =", np.round(pn, 4), " p TF =", P_TF,
      " error [mm] =", np.round(1000 * np.linalg.norm(pn - P_TF), 3))

# ---------------------------------------------------------------- comprobaciones
print("\n== Comprobaciones (V-D) ==")
print("||R^T R - I|| =", np.linalg.norm(Rn.T @ Rn - np.eye(3)))
print("det(R)        =", np.linalg.det(Rn))
print("última fila   =", T[3])


def rot2quat(Rm):
    """Matriz de rotación -> cuaternión (x, y, z, w), w >= 0."""
    w = np.sqrt(max(0.0, 1 + np.trace(Rm))) / 2
    x, y, z = (Rm[2, 1] - Rm[1, 2]), (Rm[0, 2] - Rm[2, 0]), (Rm[1, 0] - Rm[0, 1])
    if w > 1e-6:
        x, y, z = x / (4 * w), y / (4 * w), z / (4 * w)
    else:  # rotación de 180°: usar la diagonal
        x = np.sqrt(max(0.0, 1 + Rm[0, 0] - Rm[1, 1] - Rm[2, 2])) / 2
        y = np.sqrt(max(0.0, 1 - Rm[0, 0] + Rm[1, 1] - Rm[2, 2])) / 2
        z = np.sqrt(max(0.0, 1 - Rm[0, 0] - Rm[1, 1] + Rm[2, 2])) / 2
    return np.array([x, y, z, w])


qd = rot2quat(Rn)
qd_cmp = qd if np.dot(qd, QUAT_TF) >= 0 else -qd   # q y -q son la misma rotación
print("cuaternión DH =", np.round(qd_cmp, 4), " TF =", QUAT_TF)


# ================================================================
# PASO 4: JACOBIANO POSICIONAL (Jv) Y SINGULARIDADES
# ================================================================
print("\n" + "="*50)
print("PASO 4: CÁLCULO DEL JACOBIANO POSICIONAL (Jv)")
print("="*50)

# ----------------------------------------------------------------
# 1. MÉTODO ANALÍTICO
# ----------------------------------------------------------------
# Jv_analitico = dp/dq
Jv_a = p.jacobian(q)

# ----------------------------------------------------------------
# 2. MÉTODO GEOMÉTRICO
# ----------------------------------------------------------------
Jv_g = sp.zeros(3, 6)

# Posición del efector final (o6)
o6 = T06[:3, 3]

# Variables para guardar las transformaciones acumuladas y sacar Z y O
T_acumulada = sp.eye(4)

for i in range(6):
    # Extraer el eje z(i-1) (columna 2 de la matriz de rotación)
    z_i_minus_1 = T_acumulada[:3, 2]
    
    # Extraer el origen o(i-1) (columna 3 de la matriz homogénea)
    o_i_minus_1 = T_acumulada[:3, 3]
    
    # Producto cruz para articulaciones revolutas: z_(i-1) x (o_6 - o_(i-1))
    columna_geom = z_i_minus_1.cross(o6 - o_i_minus_1)
    
    # Asignar a la columna i del Jacobiano Geométrico
    Jv_g[:, i] = columna_geom
    
    # Acumular la transformación para el siguiente eslabón
    T_acumulada = T_acumulada * A[i]

# Simplificar la matriz geométrica
Jv_g = simp(Jv_g)

# ----------------------------------------------------------------
# 3. COMPROBACIONES (Caso #1)
# ----------------------------------------------------------------
# Evaluar ambos jacobianos en la configuración del Caso #1
Jv_a_num = np.array(Jv_a.subs(dict(zip(q, Q1))).evalf(), dtype=float)
Jv_g_num = np.array(Jv_g.subs(dict(zip(q, Q1))).evalf(), dtype=float)

# Calcular la diferencia (debería ser cero o cercana al error de redondeo)
diferencia_jacobianos = np.linalg.norm(Jv_a_num - Jv_g_num)

print(f"\n1. Comprobación Numérica en Caso #1:")
print(f"   Diferencia ||Jv_analítico - Jv_geométrico|| = {diferencia_jacobianos:.1e}")
if diferencia_jacobianos < 1e-6:
    print("   ✓ ÉXITO: Ambos métodos generan la misma matriz Jv.")

print(f"\n2. Análisis de la Columna 6 (wrist_3):")
print(f"   Columna 6 del Jacobiano analítico:")
sp.pprint(Jv_a[:, 5])
print("   Conclusión: Sale 0 porque el motor wrist_3 (q6) gira el efector")
print("   sobre su propio eje Z5, sin desplazar el origen O6 en x, y, o z.")

# ----------------------------------------------------------------
# 4. SINGULARIDADES (rango en q = 0)
# ----------------------------------------------------------------
# Evaluar Jv en configuración totalmente estirada (todos los q = 0)
q_zero = [0, 0, 0, 0, 0, 0]
Jv_zero_num = np.array(Jv_a.subs(dict(zip(q, q_zero))).evalf(), dtype=float)

# Calcular el rango del Jacobiano (si es menor a 3, es singularidad posicional)
rango_q_zero = np.linalg.matrix_rank(Jv_zero_num)

print(f"\n3. Análisis en q=0 (Brazo estirado horizontalmente):")
print(f"   Rango de la matriz Jv = {rango_q_zero} (Debería ser 3 para no ser singular)")
if rango_q_zero < 3:
    print("   ✓ SINGULARIDAD DETECTADA: Al estar estirado, el robot pierde")
    print("     grados de libertad posicionales (no puede moverse en ciertas direcciones).")
    print("   -> ADVERTENCIA PARA MARCO: No usar q=0 como semilla para la IK.")