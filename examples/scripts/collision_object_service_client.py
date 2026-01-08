#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from arm_api2_msgs.srv import AddGraspedObject
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion


class GraspedObjectServiceClient(Node):
    """
    Simple test client for the AddGraspedObject service.
    Demonstrates how to add grasped collision objects to the end effector.
    """

    def __init__(self):
        super().__init__("grasped_object_service_client")
        self.client = self.create_client(AddGraspedObject, "arm/add_grasped_object")
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        self.test_add_grasped_objects()

    def test_add_grasped_objects(self):
        """Test adding various grasped objects to the end effector."""
        
        # Test 1: Add a box grasped object
        self.get_logger().info("Test 1: Adding box grasped object...")
        self._add_grasped_box(
            object_id="grasped_object_box",
            position=Point(x=0.0, y=0.0, z=0.05),
            dimensions=[0.1, 0.1, 0.15]  # [x, y, z]
        )
        
        # Test 2: Add a sphere grasped object
        self.get_logger().info("Test 2: Adding sphere grasped object...")
        self._add_grasped_sphere(
            object_id="grasped_object_sphere",
            position=Point(x=0.0, y=0.0, z=0.08),
            radius=0.05
        )
        
        # Test 3: Add a cylinder grasped object
        self.get_logger().info("Test 3: Adding cylinder grasped object...")
        self._add_grasped_cylinder(
            object_id="grasped_object_cylinder",
            position=Point(x=0.0, y=0.0, z=0.1),
            height=0.2,
            radius=0.03
        )

        rclpy.shutdown()

    def _add_grasped_box(self, object_id: str, position: Point, dimensions: list):
        """Add a box grasped object."""
        request = AddGraspedObject.Request()
        
        # Create collision object
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.header.frame_id = "upper_right_finger"
        collision_object.operation = CollisionObject.ADD
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dimensions
        
        pose = Pose()
        pose.position = position
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        
        # Create attached collision object
        attached_object = AttachedCollisionObject()
        attached_object.link_name = "upper_right_finger"
        attached_object.object = collision_object
        
        request.grasped_object = collision_object
        request.attach_object = attached_object
        
        self._send_request(request, "Box")

    def _add_grasped_sphere(self, object_id: str, position: Point, radius: float):
        """Add a sphere grasped object."""
        request = AddGraspedObject.Request()
        
        # Create collision object
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.header.frame_id = "upper_right_finger"
        collision_object.operation = CollisionObject.ADD
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [radius]
        
        pose = Pose()
        pose.position = position
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        
        # Create attached collision object
        attached_object = AttachedCollisionObject()
        attached_object.link_name = "upper_right_finger"
        attached_object.object = collision_object
        
        request.grasped_object = collision_object
        request.attach_object = attached_object
        
        self._send_request(request, "Sphere")

    def _add_grasped_cylinder(self, object_id: str, position: Point, height: float, radius: float):
        """Add a cylinder grasped object."""
        request = AddGraspedObject.Request()
        
        # Create collision object
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.header.frame_id = "upper_right_finger"
        collision_object.operation = CollisionObject.ADD
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [height, radius]
        
        pose = Pose()
        pose.position = position
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        
        # Create attached collision object
        attached_object = AttachedCollisionObject()
        attached_object.link_name = "upper_right_finger"
        attached_object.object = collision_object
        
        request.grasped_object = collision_object
        request.attach_object = attached_object
        
        self._send_request(request, "Cylinder")

    def _send_request(self, request: AddGraspedObject.Request, shape_type: str):
        """Send the service request and handle the response."""
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"✓ {shape_type} grasped object '{request.grasped_object.id}' added successfully"
                )
            else:
                self.get_logger().error(
                    f"✗ Failed to add {shape_type} grasped object"
                )
        except Exception as e:
            self.get_logger().error(f"✗ Service call failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    client = GraspedObjectServiceClient()
    
    rclpy.spin(client)


if __name__ == "__main__":
    main()
