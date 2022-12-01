import numpy as np

class Stereo():

    def __init__(self, B=2, N=128, M=128, alpha=45) -> None:
        """
        Parameters
        ----------
        B : float, optional
            Distance between cameras (in metres). The default is ?.
        N,M : int, optional
            Number of pixels in image (in number of pixels). The default are ? x ?.
        f : float, optional
            Field of view (in radians). The default is ?.
        """
        self.B, self.N, self.M, self.alpha = B, N, M, alpha

    def get_coordinates(self, coords):
        """
        Parameters
        ----------
        coords : np.array
            Array of coordinates of cones in left and right cones. 
            Each row is of form (u_l, v_l, u_r, v_r), where u and v are the 
            horizontal and vertical pixels as measured from the top left corner.

        Returns
        -------
        [X, Y, Z]: np.array
            Coordinates of cone:
            X: horizontal
            Y: vertical
            Z: depth
            where [0, 0, 0] is the point between the two cameras
        """
        
        # Compute image coordinates of x and y
        x = coords[:, [0,2]] - self.N/2
        y = coords[:, [1,3]] - self.M/2
        # Compute disparity
        disp = x[:,0] - x[:,1]
        # Compute focal length
        f = self.N / (2 * np.tan(self.alpha/2))
        
        # Compute X, Y and Z (for X and Y, we compute two points: one estimated
        # from the left image and one from the right image)
        Z = np.reshape(f * self.B / (disp), (-1,1))
        X = x * Z / f
        Y = y * Z / f
        
        # Compute the mean of each individual X and Y computed from the 2 images
        # (Computing the mean for X removes the need to shift the individual points
        # B/2 to the left and right respectively)
        X = np.reshape(np.mean(X, axis=1), (-1,1))
        Y = np.reshape(np.mean(Y, axis=1), (-1,1))
        
        return np.hstack([X, Y, Z])
    