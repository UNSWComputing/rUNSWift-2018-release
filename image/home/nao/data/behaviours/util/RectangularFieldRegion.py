from Vector2D import Vector2D


class RectangularFieldRegion(object):
    def __init__(self, minExtent, maxExtent):
        self.minExtent = minExtent
        self.maxExtent = maxExtent

        if minExtent.x > maxExtent.x or minExtent.y > maxExtent.y:
            print('Invalid extents given in: {}'.format(self))

    def __str__(self):
        return '{}(\n  {}\n  {}\n)'.format(
            self.__class__.__name__,
            self.minExtent,
            self.maxExtent,
        )

    def pointInRegion(self, point, regionInflateAmount):
        testMin = Vector2D(self.minExtent.x - regionInflateAmount,
                           self.minExtent.y - regionInflateAmount)
        testMax = Vector2D(self.maxExtent.x + regionInflateAmount,
                           self.maxExtent.y + regionInflateAmount)
        return (testMin.x <= point.x <= testMax.x and
                testMin.y <= point.y <= testMax.y)
