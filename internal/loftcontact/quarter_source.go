package loftcontact

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// QuarterSourcePairs classifies every original cross-operand cap segment pair.
// It admits disjoint lines and one strictly crossed first-quadrant quarter arc.
// The reported count includes each charged pair visited before a refusal.
func QuarterSourcePairs(budget *proofbound.WorkBudget,
	prism, loft []sectionrecord.CurveSegment, events [][2]int) (uint64, bool, error) {
	expected := make(map[[2]int]struct{})
	var visited uint64
	for pi, p := range prism {
		for li, l := range loft {
			if err := budget.Step(); err != nil {
				return visited, false, err
			}
			visited++
			pLine, pOK := p.(sectionrecord.LineSeg)
			if !pOK {
				return visited, false, nil
			}
			switch lc := l.(type) {
			case sectionrecord.LineSeg:
				if quarterLineBoxesOverlap(pLine, lc) {
					return visited, false, nil
				}
			case sectionrecord.ArcSeg:
				if _, err := budget.StepN(16); err != nil {
					return visited, false, err
				}
				root, ok := quarterLineArcPair(pLine, lc)
				if !ok {
					return visited, false, nil
				}
				if root {
					expected[[2]int{pi, li}] = struct{}{}
				}
			default:
				return visited, false, nil
			}
		}
	}
	if len(expected) != len(events) {
		return visited, false, nil
	}
	for _, pair := range events {
		if err := budget.Step(); err != nil {
			return visited, false, err
		}
		if _, ok := expected[pair]; !ok {
			return visited, false, nil
		}
		delete(expected, pair)
	}
	return visited, len(expected) == 0, nil
}

func quarterLineBox(line sectionrecord.LineSeg) [4]float64 {
	minU, maxU := line.Start.U, line.End.U
	if minU > maxU {
		minU, maxU = maxU, minU
	}
	minV, maxV := line.Start.V, line.End.V
	if minV > maxV {
		minV, maxV = maxV, minV
	}
	return [4]float64{minU, minV, maxU, maxV}
}

func quarterBoxesOverlap(a, b [4]float64) bool {
	return !(a[2] < b[0] || b[2] < a[0] || a[3] < b[1] || b[3] < a[1])
}

func quarterLineBoxesOverlap(a, b sectionrecord.LineSeg) bool {
	return quarterBoxesOverlap(quarterLineBox(a), quarterLineBox(b))
}

func quarterLineArcPair(line sectionrecord.LineSeg,
	arc sectionrecord.ArcSeg) (bool, bool) {
	coords := [...]float64{line.Start.U, line.Start.V, line.End.U, line.End.V,
		arc.Center.U, arc.Center.V, arc.Start.U, arc.Start.V,
		arc.End.U, arc.End.V}
	var r [10]*big.Rat
	for i, v := range coords {
		r[i] = new(big.Rat).SetFloat64(v)
		if r[i] == nil || r[i].Num().BitLen() > 16 ||
			r[i].Denom().BitLen() > 16 {
			return false, false
		}
	}
	radius := new(big.Rat).Sub(r[6], r[4])
	if radius.Sign() <= 0 || r[7].Cmp(r[5]) != 0 ||
		r[8].Cmp(r[4]) != 0 ||
		radius.Cmp(new(big.Rat).Sub(r[9], r[5])) != 0 {
		return false, false
	}
	if !quarterBoxesOverlap(quarterLineBox(line),
		[4]float64{arc.Center.U, arc.Center.V, arc.Start.U, arc.End.V}) {
		return false, true
	}
	r2 := new(big.Rat).Mul(radius, radius)
	sign := func(x, y *big.Rat) int {
		dx := new(big.Rat).Sub(x, r[4])
		dy := new(big.Rat).Sub(y, r[5])
		d2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx),
			new(big.Rat).Mul(dy, dy))
		return d2.Cmp(r2)
	}
	a, b := sign(r[0], r[1]), sign(r[2], r[3])
	if a == 0 || b == 0 {
		return false, false
	}
	if a < 0 && b < 0 {
		return false, true
	}
	if a*b < 0 {
		return true, true
	}
	return false, false
}

// QuarterSourceInterval names one arranged fragment of an original cap segment.
type QuarterSourceInterval struct {
	Loft       bool
	Segment    int
	Start, End float64
}

// QuarterSourceIntervals checks that arranged fragments cover every original
// prism and Loft cap segment over its full natural [0,1] parameter range.
func QuarterSourceIntervals(budget *proofbound.WorkBudget, prismCount, loftCount int,
	fragments []QuarterSourceInterval) (bool, error) {
	type key struct {
		loft    bool
		segment int
	}
	type interval struct{ lo, hi float64 }
	all := make(map[key][]interval)
	for _, fragment := range fragments {
		if err := budget.Step(); err != nil {
			return false, err
		}
		count := prismCount
		if fragment.Loft {
			count = loftCount
		}
		if fragment.Segment < 0 || fragment.Segment >= count {
			return false, nil
		}
		k := key{loft: fragment.Loft, segment: fragment.Segment}
		if len(all[k]) >= 128 {
			return false, nil
		}
		all[k] = append(all[k], interval{fragment.Start, fragment.End})
	}
	check := func(k key) (bool, error) {
		intervals := all[k]
		if len(intervals) == 0 {
			return false, nil
		}
		for i := 1; i < len(intervals); i++ {
			j := i
			for j > 0 {
				if err := budget.Step(); err != nil {
					return false, err
				}
				if intervals[j-1].lo <= intervals[j].lo {
					break
				}
				intervals[j-1], intervals[j] = intervals[j], intervals[j-1]
				j--
			}
		}
		reach := float64(0)
		for _, v := range intervals {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if proofbound.IsNonFinite(v.lo) || proofbound.IsNonFinite(v.hi) ||
				v.lo < 0 || v.lo > reach || v.hi <= v.lo || v.hi > 1 {
				return false, nil
			}
			if v.hi > reach {
				reach = v.hi
			}
		}
		return reach == 1, nil
	}
	if len(all) != prismCount+loftCount {
		return false, nil
	}
	for i := range prismCount {
		ok, err := check(key{segment: i})
		if err != nil || !ok {
			return ok, err
		}
	}
	for i := range loftCount {
		ok, err := check(key{loft: true, segment: i})
		if err != nil || !ok {
			return ok, err
		}
	}
	return true, nil
}
