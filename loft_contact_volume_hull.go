package decad

import (
	"errors"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// The input and every intermediate rational are limited before arithmetic
// allocates their numerator and denominator. A comparison has the same limit
// on its two cross products. This is a private refusal limit, not a new work
// allowance for the enclosing output proof.
const loftHullRatBits = 8192

var errLoftHullSize = errors.New("loft hull rational size exceeds its private limit")

type loftHullMath struct{ budget *proofbound.WorkBudget }

func loftHullFits(x *big.Rat) bool {
	return x != nil && x.Num().BitLen() <= loftHullRatBits && x.Denom().BitLen() <= loftHullRatBits
}

func (m loftHullMath) step() error {
	if err := m.budget.Step(); err != nil {
		return err
	}
	return m.budget.Err()
}

func (m loftHullMath) add(a, b *big.Rat) (*big.Rat, error) {
	if err := m.step(); err != nil {
		return nil, err
	}
	an, ad, bn, bd := a.Num().BitLen(), a.Denom().BitLen(), b.Num().BitLen(), b.Denom().BitLen()
	if an+bd+1 > loftHullRatBits || bn+ad+1 > loftHullRatBits || ad+bd > loftHullRatBits {
		return nil, errLoftHullSize
	}
	r := new(big.Rat).Add(a, b)
	if !loftHullFits(r) {
		return nil, errLoftHullSize
	}
	return r, nil
}

func (m loftHullMath) sub(a, b *big.Rat) (*big.Rat, error) {
	if err := m.step(); err != nil {
		return nil, err
	}
	an, ad, bn, bd := a.Num().BitLen(), a.Denom().BitLen(), b.Num().BitLen(), b.Denom().BitLen()
	if an+bd+1 > loftHullRatBits || bn+ad+1 > loftHullRatBits || ad+bd > loftHullRatBits {
		return nil, errLoftHullSize
	}
	r := new(big.Rat).Sub(a, b)
	if !loftHullFits(r) {
		return nil, errLoftHullSize
	}
	return r, nil
}

func (m loftHullMath) mul(a, b *big.Rat) (*big.Rat, error) {
	if err := m.step(); err != nil {
		return nil, err
	}
	if a.Num().BitLen()+b.Num().BitLen() > loftHullRatBits ||
		a.Denom().BitLen()+b.Denom().BitLen() > loftHullRatBits {
		return nil, errLoftHullSize
	}
	r := new(big.Rat).Mul(a, b)
	if !loftHullFits(r) {
		return nil, errLoftHullSize
	}
	return r, nil
}

func (m loftHullMath) cmp(a, b *big.Rat) (int, error) {
	if err := m.step(); err != nil {
		return 0, err
	}
	if a.Num().BitLen()+b.Denom().BitLen() > loftHullRatBits ||
		b.Num().BitLen()+a.Denom().BitLen() > loftHullRatBits {
		return 0, errLoftHullSize
	}
	return a.Cmp(b), nil
}

func (m loftHullMath) sign(a *big.Rat) (int, error) {
	if err := m.step(); err != nil {
		return 0, err
	}
	return a.Sign(), nil
}

func (m loftHullMath) delta(a, b [3]*big.Rat) ([3]*big.Rat, error) {
	var d [3]*big.Rat
	for i := range d {
		v, err := m.sub(a[i], b[i])
		if err != nil {
			return d, err
		}
		d[i] = v
	}
	return d, nil
}

func (m loftHullMath) cross(a, b [3]*big.Rat) ([3]*big.Rat, error) {
	var out [3]*big.Rat
	for i := range out {
		j, k := (i+1)%3, (i+2)%3
		left, err := m.mul(a[j], b[k])
		if err != nil {
			return out, err
		}
		right, err := m.mul(a[k], b[j])
		if err != nil {
			return out, err
		}
		out[i], err = m.sub(left, right)
		if err != nil {
			return out, err
		}
	}
	return out, nil
}

func (m loftHullMath) dot(a, b [3]*big.Rat) (*big.Rat, error) {
	sum := big.NewRat(0, 1)
	for i := range a {
		term, err := m.mul(a[i], b[i])
		if err != nil {
			return nil, err
		}
		sum, err = m.add(sum, term)
		if err != nil {
			return nil, err
		}
	}
	return sum, nil
}

// loftVolumeHullBound returns the exact volume of the convex hull of six
// matched triangle endpoint positions. This nonnegative rational is also a
// conservative upper bound on the volume swept by that triangle. Each point
// must contain three non-nil exact rationals. A nil budget or a size refusal
// returns (nil, false, nil); a budget or cancellation error returns that error.
// The caller must still prove source occupancy and both endpoint shells.
func loftVolumeHullBound(budget *proofbound.WorkBudget, points [6][3]*big.Rat) (*big.Rat, bool, error) {
	if budget == nil {
		return nil, false, nil
	}
	m := loftHullMath{budget: budget}
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	for _, p := range points {
		for _, c := range p {
			if err := m.step(); err != nil {
				return nil, false, err
			}
			if !loftHullFits(c) {
				return nil, false, nil
			}
		}
	}
	volume, err := m.volume(points)
	if errors.Is(err, errLoftHullSize) {
		return nil, false, nil
	}
	if err != nil {
		return nil, false, err
	}
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	return volume, true, nil
}

func (m loftHullMath) volume(points [6][3]*big.Rat) (*big.Rat, error) {
	var center [3]*big.Rat
	for axis := range center {
		sum := big.NewRat(0, 1)
		for _, p := range points {
			var err error
			sum, err = m.add(sum, p[axis])
			if err != nil {
				return nil, err
			}
		}
		var err error
		center[axis], err = m.mul(sum, big.NewRat(1, 6))
		if err != nil {
			return nil, err
		}
	}

	// A supporting plane is identified by all input points on it. This
	// deduplicates a quadrilateral face found by each of its triples.
	// Each ordered point difference is exact and independent of the candidate
	// plane. Reuse it across triples while charging its arithmetic only once.
	var differences [6][6][3]*big.Rat
	difference := func(from, to int) ([3]*big.Rat, error) {
		if differences[from][to][0] != nil {
			return differences[from][to], nil
		}
		value, err := m.delta(points[to], points[from])
		if err != nil {
			return value, err
		}
		differences[from][to] = value
		return value, nil
	}
	seen := make(map[uint8]struct{}, 20)
	total := big.NewRat(0, 1)
	for i := range 6 {
		for j := i + 1; j < 6; j++ {
			for k := j + 1; k < 6; k++ {
				if err := m.budget.Err(); err != nil {
					return nil, err
				}
				a, err := difference(i, j)
				if err != nil {
					return nil, err
				}
				b, err := difference(i, k)
				if err != nil {
					return nil, err
				}
				normal, err := m.cross(a, b)
				if err != nil {
					return nil, err
				}
				drop := -1
				for axis, c := range normal {
					sign, signErr := m.sign(c)
					if signErr != nil {
						return nil, signErr
					}
					if sign != 0 && drop < 0 {
						drop = axis
					}
				}
				if drop < 0 {
					continue
				}
				var mask uint8
				positive, negative := false, false
				for n := range points {
					if n == i {
						mask |= 1 << n
						continue
					}
					d, deltaErr := difference(i, n)
					if deltaErr != nil {
						return nil, deltaErr
					}
					side, dotErr := m.dot(normal, d)
					if dotErr != nil {
						return nil, dotErr
					}
					sign, signErr := m.sign(side)
					if signErr != nil {
						return nil, signErr
					}
					switch sign {
					case 0:
						mask |= 1 << n
					case 1:
						positive = true
					case -1:
						negative = true
					}
					if positive && negative {
						break
					}
				}
				if positive && negative {
					continue
				}
				if _, found := seen[mask]; found {
					continue
				}
				seen[mask] = struct{}{}
				face, faceErr := m.faceVolume(points, center, mask, drop)
				if faceErr != nil {
					return nil, faceErr
				}
				total, err = m.add(total, face)
				if err != nil {
					return nil, err
				}
			}
		}
	}
	return m.mul(total, big.NewRat(1, 6))
}

func loftHullAxes(drop int) (int, int) {
	switch drop {
	case 0:
		return 1, 2
	case 1:
		return 0, 2
	default:
		return 0, 1
	}
}

func (m loftHullMath) projectedLess(points [6][3]*big.Rat, a, b, x, y int) (bool, error) {
	cmp, err := m.cmp(points[a][x], points[b][x])
	if err != nil {
		return false, err
	}
	if cmp != 0 {
		return cmp < 0, nil
	}
	cmp, err = m.cmp(points[a][y], points[b][y])
	return cmp < 0, err
}

func (m loftHullMath) projectedTurn(points [6][3]*big.Rat, a, b, c, x, y int) (int, error) {
	abx, err := m.sub(points[b][x], points[a][x])
	if err != nil {
		return 0, err
	}
	aby, err := m.sub(points[b][y], points[a][y])
	if err != nil {
		return 0, err
	}
	acx, err := m.sub(points[c][x], points[a][x])
	if err != nil {
		return 0, err
	}
	acy, err := m.sub(points[c][y], points[a][y])
	if err != nil {
		return 0, err
	}
	left, err := m.mul(abx, acy)
	if err != nil {
		return 0, err
	}
	right, err := m.mul(aby, acx)
	if err != nil {
		return 0, err
	}
	turn, err := m.sub(left, right)
	if err != nil {
		return 0, err
	}
	return m.sign(turn)
}

func (m loftHullMath) faceVolume(points [6][3]*big.Rat, center [3]*big.Rat,
	mask uint8, drop int) (*big.Rat, error) {
	x, y := loftHullAxes(drop)
	ids := make([]int, 0, 6)
	for i := range points {
		if mask&(1<<i) != 0 {
			ids = append(ids, i)
		}
	}
	for i := 1; i < len(ids); i++ {
		for j := i; j > 0; j-- {
			less, err := m.projectedLess(points, ids[j], ids[j-1], x, y)
			if err != nil {
				return nil, err
			}
			if !less {
				break
			}
			ids[j], ids[j-1] = ids[j-1], ids[j]
		}
	}
	unique := ids[:0]
	for _, id := range ids {
		if len(unique) == 0 {
			unique = append(unique, id)
			continue
		}
		last := unique[len(unique)-1]
		cx, err := m.cmp(points[id][x], points[last][x])
		if err != nil {
			return nil, err
		}
		cy, err := m.cmp(points[id][y], points[last][y])
		if err != nil {
			return nil, err
		}
		if cx != 0 || cy != 0 {
			unique = append(unique, id)
		}
	}
	if len(unique) < 3 {
		return nil, errLoftHullSize // a non-collinear support triple must survive projection
	}
	var lower, upper []int
	for _, id := range unique {
		for len(lower) >= 2 {
			turn, err := m.projectedTurn(points, lower[len(lower)-2], lower[len(lower)-1], id, x, y)
			if err != nil {
				return nil, err
			}
			if turn > 0 {
				break
			}
			lower = lower[:len(lower)-1]
		}
		lower = append(lower, id)
	}
	for _, id := range slices.Backward(unique) {
		for len(upper) >= 2 {
			turn, err := m.projectedTurn(points, upper[len(upper)-2], upper[len(upper)-1], id, x, y)
			if err != nil {
				return nil, err
			}
			if turn > 0 {
				break
			}
			upper = upper[:len(upper)-1]
		}
		upper = append(upper, id)
	}
	polygon := append(lower[:len(lower)-1], upper[:len(upper)-1]...)
	if len(polygon) < 3 {
		return nil, errLoftHullSize
	}
	base, err := m.delta(points[polygon[0]], center)
	if err != nil {
		return nil, err
	}
	volume6 := big.NewRat(0, 1)
	for i := 1; i+1 < len(polygon); i++ {
		if err := m.budget.Err(); err != nil {
			return nil, err
		}
		a, err := m.delta(points[polygon[i]], center)
		if err != nil {
			return nil, err
		}
		b, err := m.delta(points[polygon[i+1]], center)
		if err != nil {
			return nil, err
		}
		cross, err := m.cross(a, b)
		if err != nil {
			return nil, err
		}
		term, err := m.dot(base, cross)
		if err != nil {
			return nil, err
		}
		if err := m.step(); err != nil {
			return nil, err
		}
		term = new(big.Rat).Abs(term)
		volume6, err = m.add(volume6, term)
		if err != nil {
			return nil, err
		}
	}
	return volume6, nil
}
