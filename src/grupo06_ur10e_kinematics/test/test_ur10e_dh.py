"""Pruebas del modelo cinemático (ur10e_dh.py).

Uso rápido (sin colcon), desde la raíz del workspace:
  python3 src/grupo06_ur10e_kinematics/test/test_ur10e_dh.py
También lo ejecuta `colcon test` (pytest).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from grupo06_ur10e_kinematics import ur10e_dh as m  # noqa: E402

# Caso de validación #1 (docs/validacion_fk.md): q y T obtenidos de TF base -> tool0
Q_CASO1 = [-4.273603522042736, -1.0610410581420684, -1.6986263087082847,
           -4.805964006705923, -1.3640204123167399, 0.44281597917836635]
T_TF_CASO1 = np.array([[-0.997, 0.022, 0.068, 0.116],
                       [0.044, 0.940, 0.339, 0.219],
                       [-0.056, 0.341, -0.938, 0.785],
                       [0.0, 0.0, 0.0, 1.0]])
QUAT_TF_CASO1 = np.array([0.017, 0.984, 0.173, 0.031])  # (x, y, z, w)


class Omitida(Exception):
    """Prueba que todavía no puede ejecutarse (falta la tabla DH)."""


def requiere_tabla():
    if len(m.DH_TABLE) == 6:
        return
    try:
        import pytest
    except ImportError:
        raise Omitida('DH_TABLE todavía vacía')
    pytest.skip('DH_TABLE todavía vacía')


def test_1_dh_identidad():
    """Con todos los parámetros en 0, A debe ser la identidad."""
    assert np.allclose(m.dh_matrix(0.0, 0.0, 0.0, 0.0), np.eye(4))


def test_2_dh_traslacion():
    """theta=0.3, d=0.5, a=0.2, alpha=0: traslación = (a cos th, a sin th, d)."""
    A = m.dh_matrix(0.3, 0.5, 0.2, 0.0)
    assert A.shape == (4, 4)
    assert np.allclose(A[:3, 3], [0.2 * np.cos(0.3), 0.2 * np.sin(0.3), 0.5])


def test_3_dh_rotacion_valida():
    """Para parámetros cualesquiera: R ortonormal, det(R)=1 y última fila [0 0 0 1]."""
    A = m.dh_matrix(0.7, 0.1, -0.4, -1.2)
    R = A[:3, :3]
    assert np.allclose(R.T @ R, np.eye(3))
    assert np.isclose(np.linalg.det(R), 1.0)
    assert np.allclose(A[3], [0, 0, 0, 1])


def test_4_dh_alpha():
    """theta=0, alpha=pi/2: el eje z nuevo debe quedar en -y del anterior."""
    A = m.dh_matrix(0.0, 0.0, 0.0, np.pi / 2)
    assert np.allclose(A[:3, 2], [0, -1, 0])


def test_5_fk_all_estructura():
    """forward_kinematics_all devuelve 7 matrices: T00 = I y la última = T06."""
    requiere_tabla()
    Ts = m.forward_kinematics_all(np.zeros(6))
    assert len(Ts) == 7
    assert np.allclose(Ts[0], np.eye(4))
    assert np.allclose(Ts[-1], m.forward_kinematics(np.zeros(6)))


def test_6_fk_caso1_contra_tf():
    """La FK con DH debe reproducir el caso #1 de TF (tf2_echo da 3 decimales)."""
    requiere_tabla()
    T = m.forward_kinematics(Q_CASO1)
    err_p = np.linalg.norm(T[:3, 3] - T_TF_CASO1[:3, 3])
    print(f'    p_calc = {np.round(T[:3, 3], 4)}  error posición = {err_p:.5f} m')
    assert err_p < 2e-3
    assert np.allclose(T[:3, :3], T_TF_CASO1[:3, :3], atol=2e-3)


def test_7_rotacion_caso1():
    """Ecuaciones 11-12: R^T R = I y det(R) = 1 en el caso #1."""
    requiere_tabla()
    R = m.forward_kinematics(Q_CASO1)[:3, :3]
    assert np.allclose(R.T @ R, np.eye(3))
    assert np.isclose(np.linalg.det(R), 1.0)


def test_8_jacobiano_vs_diferencias_finitas():
    """Jv geométrico debe coincidir con la derivada numérica de p(q)."""
    requiere_tabla()
    q = np.array(Q_CASO1)
    J = m.position_jacobian(q)
    assert J.shape == (3, 6)
    h = 1e-6
    J_num = np.zeros((3, 6))
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = h
        J_num[:, i] = (m.forward_kinematics(q + dq)[:3, 3]
                       - m.forward_kinematics(q - dq)[:3, 3]) / (2 * h)
    print(f'    max |J - J_num| = {np.max(np.abs(J - J_num)):.2e}')
    assert np.allclose(J, J_num, atol=1e-6)


def test_9_cuaternion_casos_conocidos():
    """Identidad -> (0,0,0,1); Rz(90°) -> (0,0,sin45,cos45); Rx(180°) -> (1,0,0,0)."""
    def igual(qa, qb):  # q y -q representan la misma rotación
        qa, qb = np.asarray(qa, float), np.asarray(qb, float)
        return np.allclose(qa, qb, atol=1e-9) or np.allclose(qa, -qb, atol=1e-9)
    assert igual(m.rotation_to_quaternion(np.eye(3)), [0, 0, 0, 1])
    Rz = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)
    s = np.sqrt(0.5)
    assert igual(m.rotation_to_quaternion(Rz), [0, 0, s, s])
    Rx180 = np.diag([1.0, -1.0, -1.0])  # traza = -1: prueba la rama sin dividir por ~0
    assert igual(m.rotation_to_quaternion(Rx180), [1, 0, 0, 0])


def test_10_cuaternion_caso1():
    """El cuaternión del caso #1 debe coincidir con el de TF (salvo signo)."""
    requiere_tabla()
    qc = np.asarray(m.rotation_to_quaternion(m.forward_kinematics(Q_CASO1)[:3, :3]))
    print(f'    quat_calc = {np.round(qc, 3)}')
    assert (np.allclose(qc, QUAT_TF_CASO1, atol=3e-3)
            or np.allclose(qc, -QUAT_TF_CASO1, atol=3e-3))


if __name__ == '__main__':
    pruebas = [(n, f) for n, f in sorted(globals().items(),
               key=lambda kv: int(kv[0].split('_')[1]) if kv[0].startswith('test_') else 0)
               if n.startswith('test_') and callable(f)]
    ok = 0
    for nombre, f in pruebas:
        try:
            f()
            print(f'PASA   {nombre}')
            ok += 1
        except NotImplementedError:
            print(f'FALTA  {nombre}  (función sin implementar)')
        except Omitida:
            print(f'ESPERA {nombre}  (falta llenar DH_TABLE)')
        except Exception as e:  # noqa: BLE001
            print(f'FALLA  {nombre}  -> {type(e).__name__}: {e}')
    print(f'\n{ok}/{len(pruebas)} pruebas pasan')
