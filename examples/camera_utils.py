import numpy as np
from scipy.spatial.transform import Rotation as R

def xyaxes_to_quat(xyaxes):
    """
    Convert MuJoCo xyaxes format to quaternion
    
    Args:
        xyaxes (list or np.array): 6-element array [x_axis_x, x_axis_y, x_axis_z, y_axis_x, y_axis_y, y_axis_z]
        
    Returns:
        list: quaternion in [w, x, y, z] format (for robosuite)
    """
    xyaxes = np.array(xyaxes)
    
    # Extract x and y axes
    x_axis = xyaxes[:3]
    y_axis = xyaxes[3:]
    
    # Normalize axes
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    
    # Calculate z axis using cross product
    z_axis = np.cross(x_axis, y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)
    
    # Construct rotation matrix
    rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
    
    # Convert to quaternion using scipy
    rotation = R.from_matrix(rotation_matrix)
    quat_wxyz = rotation.as_quat()  # scipy returns [x,y,z,w]
    
    # Convert from [x,y,z,w] to [w,x,y,z] format for robosuite
    quat_robosuite = [quat_wxyz[3], quat_wxyz[0], quat_wxyz[1], quat_wxyz[2]]
    
    return quat_robosuite

def mujoco_camera_to_robosuite(pos_str, xyaxes_str):
    """
    Convert MuJoCo camera parameters to robosuite format
    
    Args:
        pos_str (str): Position string like "-1.517 0.006 2.304"
        xyaxes_str (str): Xyaxes string like "-0.004 -1.000 0.000 0.652 -0.002 0.758"
        
    Returns:
        tuple: (pos_list, quat_list) for robosuite set_camera
    """
    # Parse position
    pos = [float(x) for x in pos_str.split()]
    
    # Parse xyaxes
    xyaxes = [float(x) for x in xyaxes_str.split()]
    
    # Convert to quaternion
    quat = xyaxes_to_quat(xyaxes)
    
    return pos, quat

if __name__ == "__main__":
    # Test with the provided camera parameters
    
    # Camera 1
    pos1_str = "-1.517 0.006 2.304"
    xyaxes1_str = "-0.004 -1.000 0.000 0.652 -0.002 0.758"
    pos1, quat1 = mujoco_camera_to_robosuite(pos1_str, xyaxes1_str)
    
    print("Camera 1:")
    print(f"  pos={pos1}")
    print(f"  quat={quat1}")
    
    # Camera 2  
    pos2_str = "1.624 0.019 2.168"
    xyaxes2_str = "-0.012 1.000 -0.000 -0.584 -0.007 0.812"
    pos2, quat2 = mujoco_camera_to_robosuite(pos2_str, xyaxes2_str)
    
    print("\nCamera 2:")
    print(f"  pos={pos2}")
    print(f"  quat={quat2}") 