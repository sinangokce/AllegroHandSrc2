import unittest
from allegro_hand.liballegro import AllegroClient
from sensor_msgs.msg import JointState


class MockPublisher(object):
    """
    Mock publisher: just counts the number of messages that were sent out.
    """
    def __init__(self):
        self._pub_count = 0
        self._last_published = None

    def publish(self, args):
        self._pub_count += 1
        self._last_published = args
    pass


class TestAllegro(unittest.TestCase):

    def setUp(self):
        self.client = AllegroClient()
        self.client.pub_grasp = MockPublisher()
        self.client.pub_joint = MockPublisher()
        self.client.pub_envelop_torque = MockPublisher()

    def test_instantiate(self):
        self.assertIsNotNone(self.client)
        self.assertEqual(0, self.client.pub_grasp._pub_count)
        self.assertEqual(0, self.client.pub_joint._pub_count)
        self.assertEqual(0, self.client.pub_envelop_torque._pub_count)

    def test_send_hand_configuration(self):
        ret = self.client.command_hand_configuration('envelop')
        self.assertEqual(True, ret)
        self.assertEqual(1, self.client.pub_grasp._pub_count)
        self.assertEqual('envelop', self.client.pub_grasp._last_published.data)

    def test_send_invalid_hand_config(self):
        ret = self.client.command_hand_configuration('garbage')
        self.assertEqual(False, ret)
        self.assertEqual(0, self.client.pub_grasp._pub_count)

    def test_list_hand_configs(self):
        ret = self.client.list_hand_configurations()
        self.assertTrue(ret)  # Have something.
        self.assertIn('three_finger_grasp', ret)  # Have the human-readable one.

    def test_command_envelop_torque(self):
        ret = self.client.set_envelop_torque(0.5)
        self.assertTrue(ret)
        self.assertEqual(1, self.client.pub_envelop_torque._pub_count)
        published_value = self.client.pub_envelop_torque._last_published.data
        self.assertEqual(0.5, published_value)

    def test_command_large_envelop_torque(self):
        ret = self.client.set_envelop_torque(1.5)
        self.assertTrue(ret)
        self.assertEqual(1, self.client.pub_envelop_torque._pub_count)
        published_value = self.client.pub_envelop_torque._last_published.data
        self.assertEqual(1.0, published_value)

    def test_command_small_envelop_torque(self):
        ret = self.client.set_envelop_torque(-0.5)
        self.assertTrue(ret)
        self.assertEqual(1, self.client.pub_envelop_torque._pub_count)
        published_value = self.client.pub_envelop_torque._last_published.data
        self.assertEqual(0.0, published_value)

    def test_command_pose(self):
        des_pose = [0.123] * 16
        ret = self.client.command_joint_position(des_pose)
        self.assertTrue(ret)
        self.assertEqual(1, self.client.pub_joint._pub_count)
        published_state = self.client.pub_joint._last_published

        ref_state = JointState()
        ref_state.position = des_pose
        self.assertEqual(ref_state, published_state)

    def test_command_pose_wrong_dimensions(self):
        des_pose = [0.123] * 2  # Should be 16-dim
        ret = self.client.command_joint_position(des_pose)
        self.assertFalse(ret)
        self.assertEqual(0, self.client.pub_joint._pub_count)
        published_state = self.client.pub_joint._last_published
        self.assertIsNone(published_state)

    def test_command_pose_int_not_array(self):
        des_pose = 0.123  # Not even an iterable array.
        ret = self.client.command_joint_position(des_pose)
        self.assertFalse(ret)
        self.assertEqual(0, self.client.pub_joint._pub_count)
        published_state = self.client.pub_joint._last_published
        self.assertIsNone(published_state)

    def test_poll_position(self):
        joints = self.client.poll_joint_position(wait=False)
        self.assertIsNone(joints)

    def test_topic_prefix(self):
        client = AllegroClient(hand_topic_prefix='/Prefix')
        self.assertEqual('/Prefix/lib_cmd', client.pub_grasp.name)

    def test_topic_prefix_trailing_slash(self):
        client = AllegroClient(hand_topic_prefix='/Prefix/')
        self.assertEqual('/Prefix/lib_cmd', client.pub_grasp.name)

    def test_command_torques(self):
        des_torques = [0.123] * 16
        ret = self.client.command_joint_torques(des_torques)
        self.assertTrue(ret)
        self.assertEqual(1, self.client.pub_joint._pub_count)
        published_state = self.client.pub_joint._last_published

        ref_state = JointState()
        ref_state.effort = des_torques
        self.assertEqual(ref_state, published_state)

    def test_command_torques_wrong_dimensions(self):
        des_torques = [0.123] * 2  # Should be 16-dim
        ret = self.client.command_joint_torques(des_torques)
        self.assertFalse(ret)
        self.assertEqual(0, self.client.pub_joint._pub_count)
        published_state = self.client.pub_joint._last_published
        self.assertIsNone(published_state)

    def test_command_torques_int_not_array(self):
        des_torques = 0.123  # Not even an iterable array.
        ret = self.client.command_joint_torques(des_torques)
        self.assertFalse(ret)
        self.assertEqual(0, self.client.pub_joint._pub_count)
        published_state = self.client.pub_joint._last_published
        self.assertIsNone(published_state)

    def test_disconnect(self):
        self.client.disconnect()
        self.assertEqual(1, self.client.pub_grasp._pub_count)
        self.assertEqual('off', self.client.pub_grasp._last_published.data)
