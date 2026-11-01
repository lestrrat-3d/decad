package triangulation

import (
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type fanIntervalPoint struct{ x, y meshbool.FloatInterval }

// These refusals are constructed once so rejected replay does not allocate.
var (
	errConvexFanReplay1 = fmt.Errorf("%w: invalid convex fan", decaderr.ErrUnsupported)
	errConvexFanReplay2 = fmt.Errorf("%w: invalid convex fan index", decaderr.ErrUnsupported)
	errConvexFanReplay3 = fmt.Errorf("%w: unproved convex fan half-plane", decaderr.ErrUnsupported)
	errConvexFanReplay4 = fmt.Errorf("%w: unproved convex fan ray order", decaderr.ErrUnsupported)
	errConvexFanReplay5 = fmt.Errorf("%w: unproved convex fan turn", decaderr.ErrUnsupported)
	errConvexFanReplay6 = fmt.Errorf("%w: rounded cap turn", decaderr.ErrUnsupported)
)

func fanVector(a, b Point2) fanIntervalPoint {
	return fanIntervalPoint{meshbool.FivPoint(a.U).Sub(meshbool.FivPoint(b.U)), meshbool.FivPoint(a.V).Sub(meshbool.FivPoint(b.V))}
}

// ConvexFanReplay accepts a strictly convex counterclockwise polygon only when
// outward interval tests prove its radial order and every cyclic turn. Its
// triangles have the same order as EarClipReplay for an accepted polygon.
func ConvexFanReplay(ctx context.Context, pts []Point2, poly []int, b *proofbound.ReplayBudget) ([][3]int, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	n := len(poly)
	if b == nil || n < 3 || n > 1<<14 || len(pts) > 1<<14 {
		return nil, errConvexFanReplay1
	}
	// Setup constructs eight interval points, four interval differences,
	// and two interval sums. Each operation costs at most 128 scalar steps;
	// 2048 also covers the fixed validation and indexing checks.
	if err := b.SpendWork(ctx, 2048); err != nil {
		return nil, err
	}
	if err := b.ReserveRecords(ctx, uint64(len(pts)+n-2)); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(len(pts)), 1); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(n-2), uint64(unsafe.Sizeof([3]int{}))); err != nil {
		return nil, err
	}
	seen := make([]bool, len(pts))
	for _, id := range poly {
		if err := b.SpendWork(ctx, 4); err != nil {
			return nil, err
		}
		if id < 0 || id >= len(pts) || seen[id] {
			return nil, errConvexFanReplay2
		}
		seen[id] = true
	}
	center := pts[poly[n-1]]
	first, last := fanVector(pts[poly[0]], center), fanVector(pts[poly[n-2]], center)
	h := fanIntervalPoint{first.x.Add(last.x), first.y.Add(last.y)}
	for i := range n - 1 {
		// Six interval operations form the ray and three form its dot
		// product; each costs at most 128 scalar steps. The determinant
		// filter costs at most 128 more, leaving 256 for control checks.
		if err := b.SpendWork(ctx, 1536); err != nil {
			return nil, err
		}
		r := fanVector(pts[poly[i]], center)
		dot := h.x.Mul(r.x).Add(h.y.Mul(r.y))
		if dot.Lo <= 0 || dot.Abstains() {
			return nil, errConvexFanReplay3
		}
		if i > 0 {
			a, b := pts[poly[i-1]], pts[poly[i]]
			if meshbool.Orient2Float([2]float64{center.U, center.V}, [2]float64{a.U, a.V}, [2]float64{b.U, b.V}) <= 0 {
				return nil, errConvexFanReplay4
			}
		}
	}
	for i := range n {
		// The exact-sign filter needs at most 128 scalar steps; the rest
		// covers Cross2, indexing and branch checks.
		if err := b.SpendWork(ctx, 256); err != nil {
			return nil, err
		}
		a, c, d := pts[poly[i]], pts[poly[(i+1)%n]], pts[poly[(i+2)%n]]
		if meshbool.Orient2Float([2]float64{a.U, a.V}, [2]float64{c.U, c.V}, [2]float64{d.U, d.V}) <= 0 || !(Cross2(a, c, d) > 0) {
			return nil, errConvexFanReplay5
		}
	}
	out := make([][3]int, 0, n-2)
	for i := range n - 2 {
		if err := b.SpendWork(ctx, 32); err != nil {
			return nil, err
		}
		a, c, d := pts[poly[n-1]], pts[poly[i]], pts[poly[i+1]]
		if !(Cross2(a, c, d) > 0) || !(Cross2(pts[poly[n-2]], a, c) > 0) {
			return nil, errConvexFanReplay6
		}
		out = append(out, [3]int{poly[n-1], poly[i], poly[i+1]})
	}
	return out, nil
}
