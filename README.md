# _1C
!Эвристика!

Хорошо работает, там где точно видны границы и между ними большое расстояние


Алгоритм:

Для всех черных точек:

    От текущей точки запустим бфс по черным точкам константной глубины.
    Запишем множество точек до которого он дошел.
    Разделим множество точек на множества с помощью еще одного бфс, так 
    чтобы каждое множество задавало "направление" в котором мы пошли от исходной точки
    Если направлений больше 2:
    Переберем пары направлений, для каждого попробуем найти в нем две точки, такие
    что они лежат на одной прямой с исходной.
    Если получилось, то исходная точка является пересечением, тк если ребра лежат на одной прямой, то это одно ребро
    после этого покрасим точку и ее окрестность в другой цвет, чтобы не посчитать ее еще раз