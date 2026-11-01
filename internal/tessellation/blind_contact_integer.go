package tessellation

import (
	"context"
	"math/bits"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// blindContactArithmetic owns the meter for serial fixed-width operations.
type blindContactArithmetic struct {
	ctx    context.Context //nolint:containedctx // The context is confined to one synchronous replay or test.
	budget *proofbound.ReplayBudget
}

func (a blindContactArithmetic) requireBits(x *blindContactInt, limit int) error {
	if x.known {
		if err := a.budget.SpendWork(a.ctx, 1); err != nil {
			return err
		}
		if x.used != 0 {
			i := int(x.used) - 1
			if i*64+64-bits.LeadingZeros64(x.limbs[i]) > limit {
				return decaderr.ErrUnsupported
			}
		}
		return nil
	}
	if err := a.budget.SpendWork(a.ctx, 32); err != nil {
		return err
	}
	for i := len(x.limbs) - 1; i >= 0; i-- {
		if x.limbs[i] != 0 {
			if i*64+64-bits.LeadingZeros64(x.limbs[i]) > limit {
				return decaderr.ErrUnsupported
			}
			return nil
		}
	}
	return nil
}

func (a blindContactArithmetic) compare(x, y *blindContactInt) (int, error) {
	n, m, err := a.widths(x, y)
	if err != nil {
		return 0, err
	}
	if err := a.budget.SpendWork(a.ctx, uint64(max(n, m)+1)); err != nil {
		return 0, err
	}
	if n == 0 && m == 0 {
		return 0, nil
	}
	xneg, yneg := n != 0 && x.negative, m != 0 && y.negative
	if xneg != yneg {
		if xneg {
			return -1, nil
		}
		return 1, nil
	}
	order := 0
	for i := max(n, m) - 1; i >= 0; i-- {
		if x.limbs[i] < y.limbs[i] {
			order = -1
			break
		}
		if x.limbs[i] > y.limbs[i] {
			order = 1
			break
		}
	}
	if xneg {
		order = -order
	}
	return order, nil
}

// normalizeContactEndpoint gives every homogeneous endpoint a positive weight.
// The operation charges the full fixed-width zero check before changing signs.
func (a blindContactArithmetic) normalizeContactEndpoint(p *[4]blindContactInt) error {
	if err := a.budget.SpendWork(a.ctx, 160); err != nil {
		return err
	}
	var nonzero [4]bool
	for i := range p {
		for _, word := range p[i].limbs {
			nonzero[i] = nonzero[i] || word != 0
		}
	}
	if !nonzero[3] {
		return decaderr.ErrUnsupported
	}
	flip := p[3].negative
	for i := range p {
		if !nonzero[i] {
			p[i].negative = false
		} else if flip {
			p[i].negative = !p[i].negative
		}
	}
	return nil
}

func (a blindContactArithmetic) widths(x, y *blindContactInt) (int, int, error) {
	cost := uint64(0)
	if x.known {
		cost++
	} else {
		cost += 32
	}
	if y.known {
		cost++
	} else {
		cost += 32
	}
	if err := a.budget.SpendWork(a.ctx, cost); err != nil {
		return 0, 0, err
	}
	n, m := int(x.used), int(y.used)
	if !x.known {
		n = len(x.limbs)
		for n > 0 && x.limbs[n-1] == 0 {
			n--
		}
	}
	if !y.known {
		m = len(y.limbs)
		for m > 0 && y.limbs[m-1] == 0 {
			m--
		}
	}
	return n, m, nil
}

type blindContactVector [3]blindContactInt

// crossingContactEndpoint intersects an edge with a plane. Distances must have
// opposite signs; all components are homogeneous integers in the lifted space.
func (a blindContactArithmetic) crossingContactEndpoint(p, q *blindContactVector,
	dp, dq *blindContactInt, out *[4]blindContactInt, products *[2]blindContactInt, zero *blindContactInt) error {
	if err := a.budget.SpendWork(a.ctx, 160); err != nil {
		return err
	}
	*out = [4]blindContactInt{}
	sp, err := a.compare(dp, zero)
	if err != nil {
		return err
	}
	sq, err := a.compare(dq, zero)
	if err != nil {
		return err
	}
	if sp == 0 || sq == 0 || sp == sq {
		return decaderr.ErrUnsupported
	}
	if err := a.requireBits(dp, 450); err != nil {
		return err
	}
	if err := a.requireBits(dq, 450); err != nil {
		return err
	}
	for i := range 3 {
		if err := a.requireBits(&p[i], 148); err != nil {
			return err
		}
		if err := a.requireBits(&q[i], 148); err != nil {
			return err
		}
		if err := a.multiplyContactInto(&products[0], dp, &q[i]); err != nil {
			return err
		}
		if err := a.multiplyContactInto(&products[1], dq, &p[i]); err != nil {
			return err
		}
		if err := a.addContactInto(&out[i], &products[0], &products[1], true); err != nil {
			return err
		}
		if err := a.requireBits(&out[i], 600); err != nil {
			return err
		}
	}
	if err := a.addContactInto(&out[3], dp, dq, true); err != nil {
		return err
	}
	if err := a.normalizeContactEndpoint(out); err != nil {
		return err
	}
	return nil
}

// clipContactTriangle returns its intersection with a non-coplanar plane.
// Every returned endpoint has strictly positive weight.
func (a blindContactArithmetic) clipContactTriangle(points *[3]blindContactVector,
	distances *[3]blindContactInt, out *[2][4]blindContactInt, products *[2]blindContactInt, zero *blindContactInt) (int, error) {
	var signs [3]int
	// Eight fixed integers are cleared and up to six are copied from points.
	if err := a.budget.SpendWork(a.ctx, 512); err != nil {
		return 0, err
	}
	*out = [2][4]blindContactInt{}
	for i := range signs {
		var err error
		signs[i], err = a.compare(&distances[i], zero)
		if err != nil {
			return 0, err
		}
	}
	if signs == [3]int{} {
		return 0, decaderr.ErrUnsupported
	}
	count := 0
	for i := range signs {
		if signs[i] != 0 {
			continue
		}
		if count == len(out) {
			return 0, decaderr.ErrUnsupported
		}
		for k := range 3 {
			out[count][k] = points[i][k]
		}
		out[count][3].limbs[0] = 1
		out[count][3].used, out[count][3].known = 1, true
		count++
	}
	for i := range signs {
		j := (i + 1) % 3
		if signs[i]*signs[j] >= 0 {
			continue
		}
		if count == len(out) {
			return 0, decaderr.ErrUnsupported
		}
		if err := a.crossingContactEndpoint(&points[i], &points[j], &distances[i], &distances[j], &out[count], products, zero); err != nil {
			return 0, err
		}
		count++
	}
	return count, nil
}

// noncoplanarContactAllowed decides interval overlap after both triangles
// have been clipped against the other plane and the shared indices counted.
func (a blindContactArithmetic) noncoplanarContactAllowed(left, right *[2][4]blindContactInt,
	leftCount, rightCount, shared int, direction *blindContactVector, scratch *blindContactScratch) (bool, error) {
	if shared < 0 || shared > 2 || leftCount < 0 || leftCount > 2 || rightCount < 0 || rightCount > 2 {
		return false, decaderr.ErrUnsupported
	}
	if err := a.budget.SpendWork(a.ctx, 128); err != nil {
		return false, err
	}
	if leftCount == 0 || rightCount == 0 {
		return shared == 0, nil
	}
	var directionNonzero uint64
	for i := range direction {
		for _, word := range direction[i].limbs {
			directionNonzero |= word
		}
	}
	if directionNonzero == 0 {
		return false, decaderr.ErrUnsupported
	}
	lp, rp := &scratch.projected[0], &scratch.projected[1]
	for i := range leftCount {
		if err := a.projectContactInto(&lp[i], &left[i], direction, scratch); err != nil {
			return false, err
		}
	}
	for i := range rightCount {
		if err := a.projectContactInto(&rp[i], &right[i], direction, scratch); err != nil {
			return false, err
		}
	}
	ll, lh, rl, rh := 0, 0, 0, 0
	if leftCount == 2 {
		order, err := a.compareContactFractionsArena(&lp[0], &left[0][3], &lp[1], &left[1][3], scratch)
		if err != nil {
			return false, err
		}
		if order <= 0 {
			lh = 1
		} else {
			ll = 1
		}
	}
	if rightCount == 2 {
		order, err := a.compareContactFractionsArena(&rp[0], &right[0][3], &rp[1], &right[1][3], scratch)
		if err != nil {
			return false, err
		}
		if order <= 0 {
			rh = 1
		} else {
			rl = 1
		}
	}
	a1, err := a.compareContactFractionsArena(&lp[lh], &left[lh][3], &rp[rl], &right[rl][3], scratch)
	if err != nil {
		return false, err
	}
	a2, err := a.compareContactFractionsArena(&rp[rh], &right[rh][3], &lp[ll], &left[ll][3], scratch)
	if err != nil {
		return false, err
	}
	if shared == 0 {
		return a1 < 0 || a2 < 0, nil
	}
	if shared == 1 {
		return a1 == 0 || a2 == 0, nil
	}
	// A caller proves that both triangles contain the two distinct indexed
	// vertices and that their planes differ; their intersection is that edge.
	return a1 >= 0 && a2 >= 0, nil
}
