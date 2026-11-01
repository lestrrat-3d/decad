package tessellation

import (
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

func (a blindContactArithmetic) orientContact2(p, q, r *blindContactVector, u, v int,
	slots *[8]blindContactInt) (int, error) {
	if err := a.addContactInto(&slots[0], &q[u], &p[u], true); err != nil {
		return 0, err
	}
	if err := a.addContactInto(&slots[1], &q[v], &p[v], true); err != nil {
		return 0, err
	}
	if err := a.addContactInto(&slots[2], &r[u], &p[u], true); err != nil {
		return 0, err
	}
	if err := a.addContactInto(&slots[3], &r[v], &p[v], true); err != nil {
		return 0, err
	}
	if err := a.multiplyContactInto(&slots[4], &slots[0], &slots[3]); err != nil {
		return 0, err
	}
	if err := a.multiplyContactInto(&slots[5], &slots[1], &slots[2]); err != nil {
		return 0, err
	}
	if err := a.addContactInto(&slots[6], &slots[4], &slots[5], true); err != nil {
		return 0, err
	}
	if err := a.requireBits(&slots[6], 300); err != nil {
		return 0, err
	}
	return a.compare(&slots[6], &slots[7])
}

func (a blindContactArithmetic) onContactSegment(p, q, r *blindContactVector, u, v int) (bool, error) {
	for _, k := range [...]int{u, v} {
		pq, err := a.compare(&p[k], &q[k])
		if err != nil {
			return false, err
		}
		pr, err := a.compare(&p[k], &r[k])
		if err != nil {
			return false, err
		}
		qr, err := a.compare(&q[k], &r[k])
		if err != nil {
			return false, err
		}
		if pq <= 0 && (pr > 0 || qr < 0) || pq > 0 && (pr < 0 || qr > 0) {
			return false, nil
		}
	}
	return true, nil
}

func contactIndexInTriangle(index int, tri [3]int) bool {
	return index == tri[0] || index == tri[1] || index == tri[2]
}

// coplanarContactAllowed checks closed edge intersections and containment in
// the projection that drops a nonzero normal component. Triangles are open.
func (a blindContactArithmetic) coplanarContactAllowed(vertices []blindContactVector, floats []r3.Vec,
	x, y [3]int, shared [2]int, sharedCount, u, v int, slots *[8]blindContactInt) (bool, error) {
	if sharedCount < 0 || sharedCount > 2 || u < 0 || u > 2 || v < 0 || v > 2 || u == v {
		return false, decaderr.ErrUnsupported
	}
	if err := a.budget.SpendWork(a.ctx, 512); err != nil {
		return false, err
	}
	if sharedCount == 2 {
		ax, ay := -1, -1
		for _, i := range x {
			if i != shared[0] && i != shared[1] {
				ax = i
			}
		}
		for _, i := range y {
			if i != shared[0] && i != shared[1] {
				ay = i
			}
		}
		if ax < 0 || ay < 0 {
			return false, decaderr.ErrUnsupported
		}
		sx, err := a.orientContact2(&vertices[shared[0]], &vertices[shared[1]], &vertices[ax], u, v, slots)
		if err != nil {
			return false, err
		}
		sy, err := a.orientContact2(&vertices[shared[0]], &vertices[shared[1]], &vertices[ay], u, v, slots)
		if err != nil {
			return false, err
		}
		if sx == 0 || sy == 0 || sx == sy {
			return false, nil
		}
		// Both triangles have the indexed edge and lie in opposite closed
		// half-planes, so their intersection is exactly that edge.
		return true, nil
	}
	for i := range 3 {
		p, q := x[i], x[(i+1)%3]
		for j := range 3 {
			r, s := y[j], y[(j+1)%3]
			if err := a.budget.SpendWork(a.ctx, 24); err != nil {
				return false, err
			}
			separated := false
			for _, axis := range [...]int{u, v} {
				p0, p1 := contactAxis(floats[p], axis), contactAxis(floats[q], axis)
				q0, q1 := contactAxis(floats[r], axis), contactAxis(floats[s], axis)
				if max(p0, p1) < min(q0, q1) || max(q0, q1) < min(p0, p1) {
					separated = true
					break
				}
			}
			if separated {
				continue
			}
			o1, err := a.orientContact2(&vertices[p], &vertices[q], &vertices[r], u, v, slots)
			if err != nil {
				return false, err
			}
			o2, err := a.orientContact2(&vertices[p], &vertices[q], &vertices[s], u, v, slots)
			if err != nil {
				return false, err
			}
			o3, err := a.orientContact2(&vertices[r], &vertices[s], &vertices[p], u, v, slots)
			if err != nil {
				return false, err
			}
			o4, err := a.orientContact2(&vertices[r], &vertices[s], &vertices[q], u, v, slots)
			if err != nil {
				return false, err
			}
			if o1*o2 < 0 && o3*o4 < 0 {
				return false, nil
			}
			for _, c := range [...]struct {
				sign, point, start, end int
				other                   [3]int
			}{
				{o1, r, p, q, x}, {o2, s, p, q, x}, {o3, p, r, s, y}, {o4, q, r, s, y},
			} {
				if c.sign != 0 {
					continue
				}
				on, err := a.onContactSegment(&vertices[c.start], &vertices[c.end], &vertices[c.point], u, v)
				if err != nil {
					return false, err
				}
				if on && !contactIndexInTriangle(c.point, c.other) {
					return false, nil
				}
			}
		}
	}
	for _, pair := range [...][2][3]int{{x, y}, {y, x}} {
		for _, p := range pair[0] {
			if contactIndexInTriangle(p, pair[1]) {
				continue
			}
			if err := a.budget.SpendWork(a.ctx, 24); err != nil {
				return false, err
			}
			outside := false
			for _, axis := range [...]int{u, v} {
				v := contactAxis(floats[p], axis)
				v0 := contactAxis(floats[pair[1][0]], axis)
				v1 := contactAxis(floats[pair[1][1]], axis)
				v2 := contactAxis(floats[pair[1][2]], axis)
				if v < min(v0, v1, v2) || v > max(v0, v1, v2) {
					outside = true
					break
				}
			}
			if outside {
				continue
			}
			var signs [3]int
			for i := range signs {
				s, err := a.orientContact2(&vertices[pair[1][i]], &vertices[pair[1][(i+1)%3]], &vertices[p], u, v, slots)
				if err != nil {
					return false, err
				}
				signs[i] = s
			}
			if (signs[0] >= 0 && signs[1] >= 0 && signs[2] >= 0) ||
				(signs[0] <= 0 && signs[1] <= 0 && signs[2] <= 0) {
				return false, nil
			}
		}
	}
	return true, nil
}
