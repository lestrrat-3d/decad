package twistsweep

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// This file refines a one-cell twisted sweep for a consumer that needs a
// closer held boundary than the sweep's own (agent-docs/twist-sweep-cut-design.md §3).
// A one-cell sweep holds each wall as two flat triangles, so its proven
// displacement is dominated by the ruled quad's warp, |edge|·|angle|/4. The
// stationed build already shortens that term by cutting the same true section
// into cells. The refinement runs that build over the same record and rebinds
// the result to the live sweep's own faces, so the sweep itself, its volume,
// its area and its identity stay as they were published.

// refinable reports the geometry of a one-cell sweep's refinement, and false
// for a sweep that is already stationed or that the stationed build would
// refuse.
func (p Payload) refinable() (poly TwistPolygon, theta float64,
	thetaIv proofbound.RatInterval, cells int, ok bool) {
	if p.Mesh != nil || p.Path == nil {
		return poly, 0, thetaIv, 0, false
	}
	poly, err := TwistPolygonOf(p.Profile)
	if err != nil || poly.HasArc {
		return poly, 0, thetaIv, 0, false
	}
	theta, err = p.Angle.In(units.Radian)
	if err != nil {
		return poly, 0, thetaIv, 0, false
	}
	thetaIv, enclosed := revolveangle.FromValue(p.Angle).Enclosure()
	if !enclosed {
		return poly, 0, thetaIv, 0, false
	}
	upper := proofbound.RatFloatUp(proofbound.IntervalAbsUpper(thetaIv))
	cells, _, err = StationedTwistCellCounts(poly, upper)
	if err != nil {
		return poly, 0, thetaIv, 0, false
	}
	return poly, theta, thetaIv, cells, true
}

// refineSlack is the relative and absolute room, in mm, RefinedDelta leaves
// for a cell Loft's rounding-sized facet departure.
const refineSlack = 1e-9

// RefinedDelta is the displacement the refinement of a one-cell sweep proves,
// and false when the sweep has none or it is no closer than the sweep's own
// Delta. It is the stationed build's per-cell departure plus refineSlack;
// RefinedMesh refuses any refinement whose own proof exceeds it, so an
// understatement here refuses and never admits.
func (p Payload) RefinedDelta() (float64, bool) {
	poly, theta, thetaIv, cells, ok := p.refinable()
	if !ok {
		return 0, false
	}
	stations, err := StraightTwistStations(p.Path.End.Z, theta, thetaIv, poly, cells)
	if err != nil {
		return 0, false
	}
	cellAngle := proofbound.RatFloatUp(new(big.Rat).Quo(proofbound.IntervalAbsUpper(thetaIv), big.NewRat(int64(cells), 1)))
	delta := 0.0
	for i := range cells {
		stationError := max(stations[i].PointError, stations[i+1].PointError)
		delta = max(delta, straightTwistCellDeparture(poly, cellAngle, stationError))
	}
	// Each cell's Loft adds a rounding-sized facet departure on top of the
	// ideal figure. The request floor leaves room for it; the mesh's own
	// published Bound is the proven one, and RefinedMesh checks it against this.
	delta = proofbound.AbsSumUpper(proofbound.ProductUpper(delta, 1+refineSlack), refineSlack)
	if proofbound.IsNonFinite(delta) || delta <= 0 || delta >= p.Delta {
		return 0, false
	}
	return delta, true
}

// RefinedMesh builds the stationed held mesh of a one-cell sweep's record and
// binds every facet to the matching live face of body, found by the face's
// source role. The sweep itself is untouched. It refuses when the sweep has no
// refinement, when the refinement's proven displacement is not the one
// RefinedDelta promised, or when a role has no live face.
func RefinedMesh(ctx context.Context, env topo.Env, p Payload, body *topo.Body) (*topo.Mesh, error) {
	poly, theta, thetaIv, cells, ok := p.refinable()
	if !ok {
		return nil, fmt.Errorf(`%w: this twisted sweep has no finer held boundary`, decaderr.ErrUnsupported)
	}
	promised, ok := p.RefinedDelta()
	if !ok {
		return nil, fmt.Errorf(`%w: this twisted sweep's refinement is no closer than its held boundary`, decaderr.ErrUnsupported)
	}
	absTheta := proofbound.IntervalAbsUpper(thetaIv)
	refined, err := evalStationedCells(ctx, env, p.Profile, p.Plane, p.Path, p.Angle,
		p.RecordArea, p.Xform, poly, theta, thetaIv, absTheta, cells, false)
	if err != nil {
		return nil, err
	}
	rp, ok := refined.Payload.(Payload)
	if !ok || rp.Mesh == nil {
		return nil, fmt.Errorf(`%w: the refined twist lost its held mesh`, decaderr.ErrUnsupported)
	}
	if rp.Delta > promised {
		return nil, fmt.Errorf(`%w: the refined twist proves %g mm, not the %g mm promised`,
			decaderr.ErrUnsupported, rp.Delta, promised)
	}
	live := map[string]*topo.Face{}
	for _, face := range body.Lumps[0].Shells[0].Faces {
		if len(face.Origins) > 0 {
			live[face.Origins[0].Role] = face
		}
	}
	mesh := *rp.Mesh
	mesh.Source = make([]*topo.Face, len(rp.Mesh.Source))
	mesh.FaceBound = make(map[*topo.Face]float64, len(rp.Mesh.FaceBound))
	rebound := map[*topo.Face]*topo.Face{}
	for i, face := range rp.Mesh.Source {
		target, seen := rebound[face]
		if !seen {
			target = live[face.Origins[0].Role]
			if target == nil {
				return nil, fmt.Errorf(`%w: the refined twist has no live face for %s`,
					decaderr.ErrUnsupported, face.Origins[0].Role)
			}
			rebound[face] = target
			mesh.FaceBound[target] = rp.Mesh.FaceBound[face]
		}
		mesh.Source[i] = target
	}
	mesh.Owner = nil
	return &mesh, nil
}
