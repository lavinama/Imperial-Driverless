import numpy as np
from cone_locations import get_coordinates

def create_dummyProjections(coords, B, N, M, alpha) -> np.ndarray:
    
    f = N / (2 * np.tan(alpha/2))
    
    pl = coords + np.array([[B/2], [0], [0]])
    pr = coords - np.array([[B/2], [0], [0]])
    print("Left coordinates: \n", pl)
    print("Right coordinates: \n", pr)
    
    projection1: np.ndarray =  f * pl[:2,:] / pl[2] + np.array([[N], [M]])/2
    projection2: np.ndarray =  f * pr[:2,:] / pr[2] + np.array([[N], [M]])/2
    projection1 = projection1.T
    projection2 = projection2.T

    return np.hstack((projection1, projection2))

def test_stereo():
    pnt = np.array([[-0.1, 0.05, 0.1, 0, 0.15],[0,0.2,0.1,-0.2,-0.1], [0.5,0.4,0.78, 0.9, 0.5]])
    prj = create_dummyProjections(coords=pnt, B=0.6, N=2034, M=1688, alpha=np.pi/2)
    print(prj)
    estimated_pnt = get_coordinates(coords = prj, B=0.6, N=2034, M=1688, alpha=np.pi/2)
    print("Estimated points: \n", estimated_pnt)
    print("Real points: \n", pnt.T)
    return np.linalg.norm(estimated_pnt.T - pnt)

if __name__ == '__main__':
    print(test_stereo())

