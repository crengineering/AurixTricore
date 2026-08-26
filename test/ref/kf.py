import struct
"""An independent 4-state Kalman filter, written from the textbook equations.

Nothing in this file was derived from fusion.c. It implements the model that
fusion.h and docs/FUSION_SPEC.md describe, and nothing else:

    state   x = [d, v_d, accelBias, measBias]     (NED, d positive DOWN)

    predict, with the NED acceleration input a and step dt:
        a_eff = a - accelBias
        d    += v_d*dt + 0.5*a_eff*dt^2
        v_d  += a_eff*dt
        accelBias unchanged                       (a random walk)
        measBias *= exp(-dt/tau)                  (mean-reverting: FusionCal.h
                                                   calls tauBaroBias the
                                                   "mean-reversion" time)

        F = [[1, dt, -dt^2/2, 0 ],
             [0,  1, -dt,     0 ],
             [0,  0,  1,      0 ],
             [0,  0,  0,      exp(-dt/tau)]]

        Q = G G' * sigmaAcc^2  with G = [dt^2/2, dt, 0, 0]'   (discrete white
                                                               noise accel)
          + q_ab * dt          on the accelBias diagonal
          + sigmaBaroRw^2 * dt on the measBias diagonal       (FusionCal.h:
                                                               "[m/sqrt(s)]")

    barometer update:   z = -(alt - alt_ref),  H = [1, 0, 0, 1],  R = sigmaBaro^2
                        (fusion.h: the barometer measures d + measBias, and
                         altitude counts UP while d counts DOWN)

    GNSS position:      H = [1, 0, 0, 0],  R = hAcc^2 * gnssPosRScale
    GNSS velocity:      H = [0, 1, 0, 0],  R = sigmaGnssVel^2

The two quantities the interface does NOT pin down -- the initial covariance and
the accelerometer-bias random walk -- are left as parameters and identified from
the C filter's own published p00 trace before the comparison starts. See
compare_kf.py; they are four scalars, and no choice of them can repair an error
in the matrix algebra itself.
"""

import math

N = 4


def zeros(n=N, m=N):
    return [[0.0] * m for _ in range(n)]


def eye(n=N):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def matmul(A, B):
    n, k, m = len(A), len(B), len(B[0])
    C = zeros(n, m)
    for i in range(n):
        Ai = A[i]
        Ci = C[i]
        for t in range(k):
            a = Ai[t]
            if a == 0.0:
                continue
            Bt = B[t]
            for j in range(m):
                Ci[j] += a * Bt[j]
    return C


def transpose(A):
    return [list(r) for r in zip(*A)]


def matadd(A, B):
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]


def scal(A, s):
    return [[v * s for v in r] for r in A]


def F_matrix(dt, tau):
    F = eye()
    F[0][1] = dt
    F[0][2] = -0.5 * dt * dt
    F[1][2] = -dt
    F[3][3] = math.exp(-dt / tau) if tau > 0.0 else 1.0
    return F


def Q_matrix(dt, sigma_acc, q_ab, sigma_rw):
    """Discrete white-noise-acceleration Q, plus the two random walks."""
    g = [0.5 * dt * dt, dt, 0.0, 0.0]
    s2 = sigma_acc * sigma_acc
    Q = [[g[i] * g[j] * s2 for j in range(N)] for i in range(N)]
    Q[2][2] += q_ab * dt
    Q[3][3] += sigma_rw * sigma_rw * dt
    return Q


def f32(x):
    """Round to IEEE single precision.

    The firmware computes in float32; this reference computes in float64. Over
    thousands of feedback steps that alone makes the two states drift apart,
    which looks exactly like an algorithmic disagreement. Enabling this turns
    the reference into a single-precision one, so any remaining difference is
    a real difference in the arithmetic rather than in the word length.
    """
    return struct.unpack("f", struct.pack("f", x))[0]


class Channel:
    """One Kalman channel: [position, velocity, accelBias, measBias]."""

    single = True          # set True to mirror the firmware's float32

    def __init__(self, P0, sigma_acc, q_ab, sigma_rw=0.0, tau=1e30):
        self.x = [0.0, 0.0, 0.0, 0.0]
        self.P = [list(r) for r in P0]
        self.sigma_acc = sigma_acc
        self.q_ab = q_ab
        self.sigma_rw = sigma_rw
        self.tau = tau

    def _quantise(self):
        if not self.single:
            return
        for i in range(N):
            self.x[i] = f32(self.x[i])
        for i in range(N):
            for j in range(N):
                self.P[i][j] = f32(self.P[i][j])

    def predict(self, a, dt):
        x = self.x
        a_eff = a - x[2]
        F = F_matrix(dt, self.tau)
        x[0] += x[1] * dt + 0.5 * a_eff * dt * dt
        x[1] += a_eff * dt
        x[3] *= F[3][3]
        Q = Q_matrix(dt, self.sigma_acc, self.q_ab, self.sigma_rw)
        self.P = matadd(matmul(matmul(F, self.P), transpose(F)), Q)
        self._quantise()

    def update(self, H, z, R):
        """Scalar measurement update, Joseph form is unnecessary here because
        the reference is run in double precision."""
        P = self.P
        PH = [sum(P[i][j] * H[j] for j in range(N)) for i in range(N)]
        S = sum(H[i] * PH[i] for i in range(N)) + R
        innov = z - sum(H[i] * self.x[i] for i in range(N))
        K = [PH[i] / S for i in range(N)]
        for i in range(N):
            self.x[i] += K[i] * innov
        self.P = [[P[i][j] - K[i] * PH[j] for j in range(N)] for i in range(N)]
        # keep it exactly symmetric; the maths says it is
        for i in range(N):
            for j in range(i + 1, N):
                v = 0.5 * (self.P[i][j] + self.P[j][i])
                self.P[i][j] = self.P[j][i] = v
        self._quantise()
        return innov


PARAM_NAMES = ("P0_00", "P0_11", "P0_22", "q_accBias", "sigmaAcc^2")


def basis_predict_p00(steps, dt, tau):
    """p00 after each predict step is LINEAR in every unknown of the model,
    because a predict is an affine map on P and Q is linear in the noise
    powers. Propagate one basis matrix per unknown and return the p00 column of
    each.

    theta = [P0_00, P0_11, P0_22, q_accBias, sigmaAcc^2]

    sigmaAcc^2 is fitted rather than taken from FusionCal even though the value
    IS specified, because that is what makes the fit diagnostic: recovering the
    documented 0.09 / 0.25 confirms that the accelerometer noise enters Q the
    way the model says (G*G'*sigma^2 with G = [dt^2/2, dt, 0, 0]'). Recovering
    something else is a finding.

    The measBias states (index 3) never feed p00 during a predict -- F has no
    coupling from measBias into position -- so sigmaBaroRw and tau do not
    appear here at all.
    """
    F = F_matrix(dt, tau)
    Ft = transpose(F)

    def unit(i):
        M = zeros()
        M[i][i] = 1.0
        return M

    g = [0.5 * dt * dt, dt, 0.0, 0.0]
    dQ_acc = [[g[i] * g[j] for j in range(N)] for i in range(N)]
    dQ_ab = zeros()
    dQ_ab[2][2] = dt

    bases = [unit(0), unit(1), unit(2), zeros(), zeros()]
    increments = [None, None, None, dQ_ab, dQ_acc]

    phi = [[] for _ in bases]
    for _ in range(steps):
        for j in range(len(bases)):
            bases[j] = matmul(matmul(F, bases[j]), Ft)
            if increments[j] is not None:
                bases[j] = matadd(bases[j], increments[j])
        for j in range(len(bases)):
            phi[j].append(bases[j][0][0])
    return phi


def lstsq(A, b):
    """Least squares by normal equations; A is a list of rows."""
    n = len(A[0])
    ATA = [[sum(A[k][i] * A[k][j] for k in range(len(A))) for j in range(n)]
           for i in range(n)]
    ATb = [sum(A[k][i] * b[k] for k in range(len(A))) for i in range(n)]
    # Gaussian elimination with partial pivoting
    M = [ATA[i] + [ATb[i]] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        if abs(M[c][c]) < 1e-300:
            raise ValueError("singular normal equations")
        for r in range(n):
            if r == c:
                continue
            f = M[r][c] / M[c][c]
            for k in range(c, n + 1):
                M[r][k] -= f * M[c][k]
    return [M[i][n] / M[i][i] for i in range(n)]
