import numpy as np

class Goal:
    def __init__(self, world, quad, min_area, max_area):
        self.quad = quad
        self.in_view = False
        self.world = world
        self.min_area = min_area
        self.max_area = max_area

        coords = []
        for item in quad:
            x, y = self.world.transform_uv_to_xy(item[0], item[1])
            coords.append((x, y))

        self.coords = coords
    
    def update(self, quad):
        if self.world.maslab.robot.turn_factor<5:
            coords = []
            for item in quad:
                x, y = self.world.transform_uv_to_xy(item[0], item[1])
                coords.append((x,y))

            ############ Begin geometric checks ############

            # Correctly order points
            pts = np.array(coords)
            center = pts.mean(axis=0)
            angles = np.arctan2(pts[:,1] - center[1], pts[:,0] - center[0])
            pts = pts[np.argsort(angles)]

            # Calculate area
            x = pts[:, 0]
            y = pts[:, 1]
            area = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
            if area > self.max_area or area < self.min_area:
                return False

            # Confirm roughly equal side lengths
            diffs = np.roll(pts, -1, axis=0) - pts
            lengths =  np.linalg.norm(diffs, axis=1)
            length_ratio = np.max(lengths) / np.min(lengths)
            if length_ratio > 1.2 or length_ratio < 0.8:
                return False

            # Confirm distance
            for p in pts:
                if p[1]>300 or not self.world.bounds.is_point_in_bounds(*p):
                    return False
            
            # Confirm roughly right-angles
            angles = []
            for i in range(len(pts)):
                p0 = pts[i - 1]
                p1 = pts[i]
                p2 = pts[(i + 1) % len(pts)]

                v1 = p0 - p1
                v2 = p2 - p1

                cosang = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                angle = np.degrees(np.arccos(np.clip(cosang, -1, 1)))
                angles.append(angle)
            angles = np.array(angles)
            if not np.all(np.abs(angles - 90) < 10):
                return False

            ############ End geometric checks ############

            self.in_view = True
            self.quad = quad
            self.coords = coords
            return True
        return False
    
    def get_centroid(self):
        x = np.array([v[0] for v in self.coords])
        y = np.array([v[1] for v in self.coords])

        # close the polygon
        x2 = np.append(x, x[0])
        y2 = np.append(y, y[0])

        A = 0.5 * np.sum(x2[:-1] * y2[1:] - x2[1:] * y2[:-1])

        Cx = (1/(6*A)) * np.sum((x2[:-1] + x2[1:]) *
                                (x2[:-1] * y2[1:] - x2[1:] * y2[:-1]))
        Cy = (1/(6*A)) * np.sum((y2[:-1] + y2[1:]) *
                                (x2[:-1] * y2[1:] - x2[1:] * y2[:-1]))

        return Cx, Cy

    def closest_point_on_segment(self, p, a, b):
        """
        p = point
        a,b = segment endpoints
        """
        ap = p - a
        ab = b - a
        t = np.dot(ap, ab) / np.dot(ab, ab)
        t = np.clip(t, 0, 1)
        return a + t * ab

    def nearest_point(self):
        p = np.array((self.world.maslab.x, self.world.maslab.y))
        vertices = np.array(self.coords)

        closest = None
        min_dist = float("inf")

        for i in range(len(vertices)):
            a = vertices[i]
            b = vertices[(i+1) % len(vertices)]

            cp = self.closest_point_on_segment(p, a, b)
            dist = np.linalg.norm(p - cp)

            if dist < min_dist:
                min_dist = dist
                closest = cp

        return closest
        
    def __repr__(self):
        return f"Quad: {self.quad}\nCoords: {self.coords}\nIn view: {self.in_view}"

    