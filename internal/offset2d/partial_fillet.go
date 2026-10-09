package offset2d

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// PartialFilletContour offsets selected walks and keeps the others fixed.
// Two selected walks keep the fillet's reflex connector; a join to an
// unselected walk uses the sharp carrier intersection.
func PartialFilletContour(budget *proofbound.WorkBudget, walks []survey2d.SideWalk,
	selected, sphere []bool, amounts []float64, tol float64,
) ([]sectionrecord.CurveSegment, []Join, []int, []int, error) {
	n := len(walks)
	joins := make([]Join, n)
	for k := range n {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, nil, nil, nil, err
		}
		prev := (k + n - 1) % n
		var join Join
		var err error
		if selected[prev] && selected[k] {
			join, err = CornerJoin(walks[prev], walks[k], 1, amounts[k], tol)
		} else {
			join, err = SharpCornerJoin(walks[prev], walks[k], amounts[prev], amounts[k], tol)
		}
		if err != nil {
			return nil, nil, nil, nil, err
		}
		joins[k] = join
	}
	for i, on := range sphere {
		if !on {
			continue
		}
		pole := Point{U: walks[i].CU, V: walks[i].CV}
		next := (i + 1) % n
		joins[i].M, joins[next].M = pole, pole
	}
	capWalk, capArc := make([]int, n), make([]int, n)
	for i := range capArc {
		capWalk[i] = -1
		capArc[i] = -1
	}
	var segments []sectionrecord.CurveSegment
	for i, walk := range walks {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, nil, nil, nil, err
		}
		start, end := joins[i].M, joins[(i+1)%n].M
		if joins[i].Arc {
			start = joins[i].PB
		}
		if joins[(i+1)%n].Arc {
			end = joins[(i+1)%n].PA
		}
		if sphere[i] {
			pole := Point{U: walk.CU, V: walk.CV}
			if start != pole || end != pole {
				return nil, nil, nil, nil, fmt.Errorf(`%w: a selected sphere walk does not end at its pole`, decaderr.ErrUnsupported)
			}
			continue
		}
		if WalkConsumed(walk, start, end, tol) {
			return nil, nil, nil, nil, ErrDrop
		}
		segment, err := WalkSegment(walk, 1, amounts[i], start, end, tol)
		if err != nil {
			return nil, nil, nil, nil, err
		}
		capWalk[i] = len(segments)
		segments = append(segments, segment)
		k := (i + 1) % n
		if joins[k].Arc {
			join := joins[k]
			capArc[k] = len(segments)
			segments = append(segments, ArcSegment(
				Point{U: join.VertU, V: join.VertV}, join.PA, join.PB, false))
		}
	}
	return segments, joins, capWalk, capArc, nil
}
