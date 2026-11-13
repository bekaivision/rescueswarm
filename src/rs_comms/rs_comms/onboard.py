"""Бортовой процесс узла: агент связи и (по параметру fusion) people_fusion в одном процессе.

Launch запускает его без name=, поэтому оба узла сохраняют свои имена (comms_agent и people_fusion).
"""
import rclpy
from rclpy.executors import SingleThreadedExecutor

from rs_comms.agent import Agent


def main():
    rclpy.init()
    agent = Agent()
    nodes = [agent]
    fusion = None
    if agent.get_parameter('fusion').value:
        from rs_perception.people_fusion import PeopleFusion
        fusion = PeopleFusion()
        nodes.append(fusion)
    ex = SingleThreadedExecutor()
    for n in nodes:
        ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if fusion is not None:
            fusion._write_json()
            if fusion.det_log:
                fusion.det_log.close()
            if fusion.rgb_log:
                fusion.rgb_log.close()
        if agent.csv:
            agent.csv.close()
        for n in nodes:
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
