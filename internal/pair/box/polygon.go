package box

import "math/big"

// ClipHorizontalPolygon retains one closed half-plane using exact rational
// intersections. The clipping order fixes a stable source-feature order.
func ClipHorizontalPolygon(polygon [][2]*big.Rat, axis int, limit *big.Rat, upper bool) [][2]*big.Rat {
	if len(polygon) == 0 {
		return nil
	}
	inside := func(vertex [2]*big.Rat) bool {
		cmp := vertex[axis].Cmp(limit)
		return upper && cmp <= 0 || !upper && cmp >= 0
	}
	intersection := func(from, to [2]*big.Rat) [2]*big.Rat {
		fraction := new(big.Rat).Quo(new(big.Rat).Sub(limit, from[axis]),
			new(big.Rat).Sub(to[axis], from[axis]))
		other := 1 - axis
		result := from
		result[axis] = new(big.Rat).Set(limit)
		result[other] = new(big.Rat).Add(from[other],
			new(big.Rat).Mul(fraction, new(big.Rat).Sub(to[other], from[other])))
		return result
	}
	output := make([][2]*big.Rat, 0, len(polygon)+2)
	from := polygon[len(polygon)-1]
	fromInside := inside(from)
	for _, to := range polygon {
		toInside := inside(to)
		if fromInside != toInside {
			output = append(output, intersection(from, to))
		}
		if toInside {
			output = append(output, to)
		}
		from, fromInside = to, toInside
	}
	return output
}

// DeduplicateHorizontalPolygon removes adjacent duplicate exact vertices.
func DeduplicateHorizontalPolygon(polygon [][2]*big.Rat) [][2]*big.Rat {
	unique := make([][2]*big.Rat, 0, len(polygon))
	for _, point := range polygon {
		if len(unique) == 0 || point[0].Cmp(unique[len(unique)-1][0]) != 0 ||
			point[1].Cmp(unique[len(unique)-1][1]) != 0 {
			unique = append(unique, point)
		}
	}
	if len(unique) > 1 && unique[0][0].Cmp(unique[len(unique)-1][0]) == 0 &&
		unique[0][1].Cmp(unique[len(unique)-1][1]) == 0 {
		unique = unique[:len(unique)-1]
	}
	return unique
}

// HorizontalPolygonDoubleArea returns the exact signed double area.
func HorizontalPolygonDoubleArea(polygon [][2]*big.Rat) *big.Rat {
	area := new(big.Rat)
	for i, point := range polygon {
		next := polygon[(i+1)%len(polygon)]
		area.Add(area, new(big.Rat).Sub(new(big.Rat).Mul(point[0], next[1]),
			new(big.Rat).Mul(point[1], next[0])))
	}
	return area
}
