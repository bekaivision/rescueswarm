"""Узел slam_cloud_crop: убирает из облака лидара точки земли перед mrg_slam (Фаза 4).

Зачем: у нашего 16-лучевого лидара нижние лучи бьют в пол кольцами на расстоянии 1-18 м. Эти кольца
привязаны к самому роботу, и scan matching (GICP) сопоставляет их с кольцами ключевого кадра, получая
"нулевое смещение" там, где в кадре мало другой геометрии (например, снаружи здания вдоль одной стены).
Поза тогда застревает, пока робот едет. Без точек пола остаётся только вертикальная геометрия.
Вторая причина застревания: верх корпуса самого робота (десятки точек ближе 0.35 м от лидара) тоже
неподвижен относительно сенсора, поэтому точки ближе min_range по горизонтали тоже убираем.
Дополнительно считает, насколько геометрия кадра ограничивает движение (вырожденность, /<r>/slam_degeneracy): по
соседним точкам одного луча оценивается касательная к поверхности, из неё нормаль; матрица I = сумма n*n^T (2x2, кадр лидара
= кадр базы по осям) показывает, вдоль каких направлений scan matching вообще может что-то измерить. Вдоль длинной
прямой стены (коридор) одно собственное значение I почти ноль: смещение вдоль стены по лидару не определено.
Публикует geometry_msgs/Vector3Stamped (x=Ixx, y=Iyy, z=Ixy) со штампом облака.
Читает /<r>/lidar/points (кадр <r>/lidar_link, как есть), оставляет точки с z (в кадре лидара) в диапазоне
[min_z, max_z] и горизонтальной дальностью от min_range, публикует /<r>/slam_points тем же кадром и штампом.
Один узел на всех роботов. Параметры: robots, min_z (-0.15: лидар на 0.28 м выше пола), max_z, min_range (0.45).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.msg import ParameterDescriptor


class SlamCloudCrop(Node):
    def __init__(self):
        super().__init__('slam_cloud_crop')
        self.declare_parameter('robots', [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('min_z', -0.15)
        self.declare_parameter('max_z', 5.0)
        self.declare_parameter('min_range', 0.45)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.declare_parameter('max_step', 0.4)   # соседние точки дальше друг от друга не считаются одной поверхностью, м
        self.max_step = float(self.get_parameter('max_step').value)
        self.pubs = {}
        self.deg_pubs = {}
        robots = list(self.get_parameter('robots').value or [])
        if not robots:
            raise RuntimeError('параметр robots пуст: список роботов задаёт launch (explore.launch.py / slam.launch.py), запасного списка нет')
        for r in robots:
            self.pubs[r] = self.create_publisher(PointCloud2, f'/{r}/slam_points', 10)
            self.deg_pubs[r] = self.create_publisher(Vector3Stamped, f'/{r}/slam_degeneracy', 10)
            self.create_subscription(PointCloud2, f'/{r}/lidar/points', lambda m, r=r: self.cb(r, m), 10)

    def cb(self, r, msg):
        step = msg.point_step
        off = {f.name: f.offset for f in msg.fields}
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, step)
        col = lambda n: np.ascontiguousarray(raw[:, off[n]:off[n] + 4]).view(np.float32).ravel()  # noqa: E731
        x, y, z = col('x'), col('y'), col('z')
        # inf и nan отсекаются сами (сравнения с ними ложны)
        keep = (z >= self.min_z) & (z <= self.max_z) & (x * x + y * y >= self.min_range ** 2)
        self.publish_degeneracy(r, msg, x, y, keep)
        out = PointCloud2()
        out.header = msg.header
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = step
        out.height = 1
        out.width = int(keep.sum())
        out.row_step = step * out.width
        out.data = raw[keep].tobytes()
        out.is_dense = False
        self.pubs[r].publish(out)


    def publish_degeneracy(self, r, msg, x, y, keep):
        """I = сумма n*n^T по нормалям поверхностей в кадре (после обрезки облака)."""
        if msg.height <= 1 or msg.width * msg.height != x.size:
            return    # облако не упорядочено: оценить нормали по соседям нельзя
        h, w = msg.height, msg.width
        px = np.where(keep, x, np.nan).reshape(h, w)
        py = np.where(keep, y, np.nan).reshape(h, w)
        # касательная по соседям справа и слева в луче (по азимуту), с замыканием по кругу
        tx = np.roll(px, -1, axis=1) - np.roll(px, 1, axis=1)
        ty = np.roll(py, -1, axis=1) - np.roll(py, 1, axis=1)
        ln = np.hypot(tx, ty)
        ok = np.isfinite(ln) & (ln > 1e-3) & (ln < 2 * self.max_step)
        nx = np.where(ok, -ty / np.where(ok, ln, 1.0), 0.0)
        ny = np.where(ok, tx / np.where(ok, ln, 1.0), 0.0)
        out = Vector3Stamped()
        out.header = msg.header
        out.vector.x = float(np.sum(nx * nx))
        out.vector.y = float(np.sum(ny * ny))
        out.vector.z = float(np.sum(nx * ny))
        self.deg_pubs[r].publish(out)


def main():
    rclpy.init()
    node = SlamCloudCrop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
