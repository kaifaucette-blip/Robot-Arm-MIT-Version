(function (global) {

  const clamp = (v, min, max) => Math.max(min, Math.min(v, max));

  function identityMatrix() {
    return [
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ];
  }

  function transpose(M) {
    return M[0].map((_, c) => M.map(r => r[c]));
  }

  function matMul(A, B) {
    const rA = A.length, cA = A[0].length;
    
    // Matrix x Matrix
    if (Array.isArray(B[0])) {
      const cB = B[0].length;
      const res = Array.from({ length: rA }, () => Array(cB).fill(0));
      for (let i = 0; i < rA; i++) {
        for (let k = 0; k < cA; k++) {
          if (A[i][k] === 0) continue;
          for (let j = 0; j < cB; j++) {
            res[i][j] += A[i][k] * B[k][j];
          }
        }
      }
      return res;
    }

    // Matrix x Vector
    const res = Array(rA).fill(0);
    for (let i = 0; i < rA; i++) {
      let sum = 0;
      for (let k = 0; k < cA; k++) {
        sum += A[i][k] * B[k];
      }
      res[i] = sum;
    }
    return res;
  }

  function translationMatrix(x, y, z) {
    const T = identityMatrix();
    T[0][3] = x; T[1][3] = y; T[2][3] = z;
    return T;
  }

  function rotationMatrix(axis, theta) {
    const R = identityMatrix();
    const c = Math.cos(theta), s = Math.sin(theta);
    if (axis === 'x') {
      R[1][1] = c; R[1][2] = -s;
      R[2][1] = s; R[2][2] = c;
    } else if (axis === 'y') {
      R[0][0] = c; R[0][2] = s;
      R[2][0] = -s; R[2][2] = c;
    } else {
      R[0][0] = c; R[0][1] = -s;
      R[1][0] = s; R[1][1] = c;
    }
    return R;
  }

  function eulerToMatrix(roll, pitch, yaw) {
    const cx = Math.cos(roll), sx = Math.sin(roll);
    const cy = Math.cos(pitch), sy = Math.sin(pitch);
    const cz = Math.cos(yaw), sz = Math.sin(yaw);

    return [
      [cy * cz, -cy * sz, sy, 0],
      [sx * sy * cz + cx * sz, -sx * sy * sz + cx * cz, -sx * cy, 0],
      [-cx * sy * cz + sx * sz, cx * sy * sz + sx * cz, cx * cy, 0],
      [0, 0, 0, 1]
    ];
  }

  function poseToMatrix(p) {
    const T = translationMatrix(p.x || 0, p.y || 0, p.z || 0);
    const R = eulerToMatrix(p.roll || 0, p.pitch || 0, p.yaw || 0);
    return matMul(R, T);
  }

  function matrixToPose(T) {
    return {
      x: T[0][3],
      y: T[1][3],
      z: T[2][3],
      roll: Math.atan2(T[2][1], T[2][2]),
      pitch: Math.atan2(-T[2][0], Math.hypot(T[2][1], T[2][2])),
      yaw: Math.atan2(T[1][0], T[0][0])
    };
  }

  function solveGaussJordan(A, b) {
    const n = A.length;
    const M = A.map(row => [...row]);
    const B = [...b];

    for (let i = 0; i < n; i++) {
      let maxRow = i;
      for (let j = i + 1; j < n; j++) {
        if (Math.abs(M[j][i]) > Math.abs(M[maxRow][i])) maxRow = j;
      }

      if (Math.abs(M[maxRow][i]) < 1e-12) throw new Error('Singular matrix');

      if (maxRow !== i) {
        [M[i], M[maxRow]] = [M[maxRow], M[i]];
        [B[i], B[maxRow]] = [B[maxRow], B[i]];
      }

      const piv = M[i][i];
      for (let j = i; j < n; j++) M[i][j] /= piv;
      B[i] /= piv;

      for (let r = 0; r < n; r++) {
        if (r === i) continue;
        const factor = M[r][i];
        if (factor === 0) continue;
        for (let j = i; j < n; j++) M[r][j] -= factor * M[i][j];
        B[r] -= factor * B[i];
      }
    }
    return B;
  }

  function createRobotFromConfig(cfg = {}) {
    const joints = cfg.joints || [
      {z: 100, axis: 'z'}, {z: 120, axis: 'y'}, 
      {z: 90, axis: 'y'}, {z: 60, axis: 'x'}
    ];

    const n = joints.length;
    return {
      joints: joints.map(j => ({ x: j.x || 0, y: j.y || 0, z: j.z || 0, axis: j.axis || 'z' })),
      minAngles: cfg.minAngles || Array(n).fill(-Math.PI),
      maxAngles: cfg.maxAngles || Array(n).fill(Math.PI),
      defaultQ: cfg.defaultQ || Array(n).fill(0),
      startPose: cfg.startPose || { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 },
      toolOffset: cfg.toolOffset || { x: 0, y: 0, z: 0 }
    };
  }

  function forwardKinematics(robot, q) {
    let T = poseToMatrix(robot.startPose);
    for (let i = 0; i < robot.joints.length; i++) {
      const j = robot.joints[i];
      T = matMul(T, translationMatrix(j.x, j.y, j.z));
      T = matMul(T, rotationMatrix(j.axis, q[i] || 0));
    }
    T = matMul(T, translationMatrix(robot.toolOffset.x, robot.toolOffset.y, robot.toolOffset.z));
    return matrixToPose(T);
  }

  function jacobian(robot, q) {
    const eps = 1e-5;
    const base = forwardKinematics(robot, q);
    const J = Array.from({ length: 3 }, () => Array(robot.joints.length).fill(0));

    for (let i = 0; i < robot.joints.length; i++) {
      const q2 = [...q];
      q2[i] += eps;
      const p = forwardKinematics(robot, q2);
      J[0][i] = (p.x - base.x) / eps;
      J[1][i] = (p.y - base.y) / eps;
      J[2][i] = (p.z - base.z) / eps;
    }
    return J;
  }

  function solveDls(J, e, lambda = 0.05) {
    const JT = transpose(J);
    const JTJ = matMul(JT, J);
    const JTe = matMul(JT, e);

    const A = JTJ.map((row, i) => 
      row.map((val, j) => val + (i === j ? lambda * lambda : 0))
    );
    return solveGaussJordan(A, JTe);
  }

  function solveJacobianIK(robot, target, initialQ, opts = {}) {
    const maxIter = opts.maxIter || 100;
    const tol = opts.tolerance || 1e-3;
    const damping = opts.damping || 0.08;
    const minA = opts.minAngles || robot.minAngles;
    const maxA = opts.maxAngles || robot.maxAngles;

    let q = [...(initialQ || robot.defaultQ)];
    q = q.map((val, i) => clamp(val, minA[i], maxA[i]));

    let bestQ = [...q];
    let bestErr = Infinity;

    for (let iter = 0; iter < maxIter; iter++) {
      const pose = forwardKinematics(robot, q);
      const ex = target.x - pose.x;
      const ey = target.y - pose.y;
      const ez = target.z - pose.z;
      const errNorm = Math.hypot(ex, ey, ez);

      if (errNorm < bestErr) {
        bestErr = errNorm;
        bestQ = [...q];
      }

      if (errNorm < tol) return q;

      const J = jacobian(robot, q);
      const delta = solveDls(J, [ex, ey, ez], damping);

      let step = 1.0;
      let accepted = false;

      while (step >= 0.03125) {
        const qCand = q.map((val, i) => clamp(val + (delta[i] || 0) * step, minA[i], maxA[i]));
        const pCand = forwardKinematics(robot, qCand);
        const candErr = Math.hypot(target.x - pCand.x, target.y - pCand.y, target.z - pCand.z);

        if (candErr < errNorm) {
          q = qCand;
          accepted = true;
          break;
        }
        step *= 0.5;
      }

      if (!accepted) {
        q = q.map((val, i) => clamp(val + (delta[i] || 0) * 0.05, minA[i], maxA[i]));
      }
    }

    return bestQ;
  }

  const api = {
    createRobotFromConfig,
    forwardKinematics,
    jacobian,
    solveJacobianIK
  };

  if (typeof module !== 'undefined' && module.exports) {
    module.exports = api;
  }
  global.RobotIK = api;

})(typeof globalThis !== 'undefined' ? globalThis : this);
