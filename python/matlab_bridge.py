"""
matlab_bridge.py
Wraps MATLAB Engine API calls for the ABB IRB 1600 digital twin.

The engine is started ONCE at application launch and kept alive.
Never call start_matlab() inside a loop — it takes ~10 seconds each time.
"""

import numpy as np
import matlab.engine


class MatlabBridge:
    """Thin wrapper around matlab.engine that exposes FK / IK / Jacobian."""

    def __init__(self, matlab_folder: str):
        """
        Parameters
        ----------
        matlab_folder : str
            Absolute path to the /matlab directory containing all .m files.
        """
        print("Starting MATLAB engine — this takes ~10 s on first launch...")
        self.eng = matlab.engine.start_matlab()
        self.eng.addpath(matlab_folder, nargout=0)
        print("MATLAB engine ready.\n")

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """
        Parameters
        ----------
        q : np.ndarray, shape (6,)
            Joint angles in radians.

        Returns
        -------
        np.ndarray, shape (4, 4)
            Homogeneous transform T_base_to_ee.
        """
        q_ml = matlab.double(q.flatten().tolist())
        T_ml = self.eng.forward_kinematics(q_ml)
        return np.array(T_ml)

    def get_all_transforms(self, q: np.ndarray) -> list[np.ndarray]:
        """
        Returns T_base_to_frame_n for n = 1..6.
        Called every animation frame to reposition each link mesh.

        Parameters
        ----------
        q : np.ndarray, shape (6,)
            Joint angles in radians.

        Returns
        -------
        list of 6 np.ndarray, each shape (4, 4)
        """
        q_ml       = matlab.double(q.flatten().tolist())
        transforms = []
        for n in range(1, 7):
            T_ml = self.eng.partial_fk(q_ml, float(n))
            transforms.append(np.array(T_ml))
        return transforms

    def inverse_kinematics(self,
                           T_desired: np.ndarray,
                           q_init: np.ndarray | None = None) -> np.ndarray:
        """
        Parameters
        ----------
        T_desired : np.ndarray, shape (4, 4)
            Desired end-effector homogeneous transform.
        q_init : np.ndarray, shape (6,), optional
            Initial joint angle guess in radians.

        Returns
        -------
        np.ndarray, shape (6,)
            Solution joint angles in radians.
        """
        if q_init is None:
            q_init = np.zeros(6)

        T_ml      = matlab.double(T_desired.tolist())
        q_init_ml = matlab.double(q_init.flatten().tolist())
        q_ml      = self.eng.inverse_kinematics(T_ml, q_init_ml)
        return np.array(q_ml).flatten()

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        Parameters
        ----------
        q : np.ndarray, shape (6,)
            Joint angles in radians.

        Returns
        -------
        np.ndarray, shape (6, 6)
            Geometric Jacobian matrix.
        """
        q_ml = matlab.double(q.flatten().tolist())
        J_ml = self.eng.jacobian(q_ml)
        return np.array(J_ml)

    def close(self):
        """Shut down the MATLAB engine cleanly."""
        self.eng.quit()
        print("MATLAB engine closed.")
