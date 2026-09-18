package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file owns rotation-minimizing frame transport along a recorded Sweep
// path. Each held endpoint frame travels beside rational intervals enclosing
// the frame the exact path record denotes. Later builders can therefore use
// the held frame without mistaking its accumulated rotation rounding for exact
// geometry.

type sweepTransportFrame struct {
	frame r3.Frame

	originExact ivVec3
	uExact      ivVec3
	vExact      ivVec3
	nExact      ivVec3

	originBound float64
	uBound      float64
	vBound      float64
	nBound      float64
}

// transportSweepFramesContext returns the transported section frame at every
// path point, including the source frame. Lines translate the frame without
// rotating it. Arcs apply the right-handed carrier rotation, which is the
// rotation-minimizing transport of a frame whose normal follows the path.
func transportSweepFramesContext(
	ctx context.Context,
	path *Path,
	plane PlaneRecord,
) ([]sweepTransportFrame, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if path == nil {
		return nil, fmt.Errorf(`%w: a nil path has no frames to transport`, ErrDegenerate)
	}
	if err := validateSweepPathGeometry(path, plane); err != nil {
		return nil, err
	}
	if samePathPoint(path.Start(), path.End()) {
		return nil, fmt.Errorf(`%w: closed sweep paths have no admitted end frame`, ErrUnsupported)
	}

	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, ErrDegenerate, err)
	}
	current, err := initialSweepTransportFrame(frame)
	if err != nil {
		return nil, err
	}
	frames := make([]sweepTransportFrame, 1, len(path.records)+1)
	frames[0] = current

	for i, record := range path.records {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if record.arc == nil {
			current, err = transportSweepLine(current, record)
		} else {
			current, err = transportSweepArc(current, record)
		}
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		frames = append(frames, current)
	}
	return frames, nil
}

func initialSweepTransportFrame(frame r3.Frame) (sweepTransportFrame, error) {
	origin, okO := ivVec3Of(frame.Origin())
	u, okU := ivVec3Of(frame.U())
	v, okV := ivVec3Of(frame.V())
	n, okN := ivVec3Of(frame.N())
	if !okO || !okU || !okV || !okN {
		return sweepTransportFrame{}, fmt.Errorf(`%w: the source sweep frame is not finite`, ErrUnsupported)
	}
	return newSweepTransportFrame(frame, origin, u, v, n)
}

func transportSweepLine(current sweepTransportFrame, record pathSegmentRecord) (sweepTransportFrame, error) {
	deltaExact := ivVec3Sub(mustIVVec3Of(record.end), mustIVVec3Of(record.start))
	originExact := ivVec3Add(current.originExact, deltaExact)
	if exact, ok := exactSweepTransportFrame(
		originExact,
		current.uExact,
		current.vExact,
		current.nExact,
	); ok {
		return exact, nil
	}

	delta := record.end.Sub(record.start)
	if !finiteVec(delta) {
		return sweepTransportFrame{}, fmt.Errorf(`%w: a line span's translation is outside the representable range`, ErrUnsupported)
	}
	origin := current.frame.Origin().Add(delta)
	if !finiteVec(origin) {
		return sweepTransportFrame{}, fmt.Errorf(`%w: a line span's transported origin is outside the representable range`, ErrUnsupported)
	}
	frame, err := r3.NewFrame(origin, current.frame.U(), current.frame.V())
	if err != nil {
		return sweepTransportFrame{}, fmt.Errorf(`%w: a line span produced no finite transported frame: %s`, ErrUnsupported, err)
	}
	return newSweepTransportFrame(
		frame,
		originExact,
		current.uExact,
		current.vExact,
		current.nExact,
	)
}

func transportSweepArc(current sweepTransportFrame, record pathSegmentRecord) (sweepTransportFrame, error) {
	if record.arc == nil {
		return sweepTransportFrame{}, fmt.Errorf(`%w: an arc span holds no circular carrier`, ErrDegenerate)
	}
	arc := *record.arc
	centerExact := sweepRatIntervalVec(arc.center)
	axisRaw := sweepRatIntervalVec(arc.axis)
	axisExact, status := ivVec3Unit(axisRaw)
	if status != normalProven {
		return sweepTransportFrame{}, fmt.Errorf(`%w: an arc span has no certified axis direction`, ErrUnsupported)
	}
	sin, cos, ok := record.arcAngle.sinCosFor(record.arcPhi)
	if !ok {
		return sweepTransportFrame{}, fmt.Errorf(`%w: an arc span has no certified sine and cosine`, ErrUnsupported)
	}

	rotateDirection := func(vector ivVec3) ivVec3 {
		oneMinusCos := intervalSub(pointInterval(big.NewRat(1, 1)), cos)
		return ivVec3Add(
			ivVec3Mul(vector, cos),
			ivVec3Add(
				ivVec3Mul(ivVec3Cross(axisExact, vector), sin),
				ivVec3Mul(axisExact, intervalMul(oneMinusCos, ivVec3Dot(axisExact, vector))),
			),
		)
	}
	rotatePoint := func(point ivVec3) ivVec3 {
		return ivVec3Add(centerExact, rotateDirection(ivVec3Sub(point, centerExact)))
	}

	originExact := rotatePoint(current.originExact)
	uExact := rotateDirection(current.uExact)
	vExact := rotateDirection(current.vExact)
	nExact := rotateDirection(current.nExact)
	if exact, ok := exactSweepTransportFrame(originExact, uExact, vExact, nExact); ok {
		return exact, nil
	}

	center, ok := heldSweepRatVec(arc.center)
	if !ok {
		return sweepTransportFrame{}, fmt.Errorf(`%w: an arc span's center is outside the representable range`, ErrUnsupported)
	}
	axis, ok := heldSweepUnitVector(axisExact)
	if !ok {
		return sweepTransportFrame{}, fmt.Errorf(`%w: an arc span's axis is outside the representable range`, ErrUnsupported)
	}
	rotation, err := r3.RotationAround(center, axis, units.Radians(record.arcPhi))
	if err != nil {
		return sweepTransportFrame{}, fmt.Errorf(`%w: an arc span produced no finite rotation: %s`, ErrUnsupported, err)
	}
	origin := rotation.Apply(current.frame.Origin())
	u := rotation.ApplyDir(current.frame.U())
	v := rotation.ApplyDir(current.frame.V())
	if !finiteVec(origin) || !finiteVec(u) || !finiteVec(v) {
		return sweepTransportFrame{}, fmt.Errorf(`%w: an arc span produced a non-finite transported frame`, ErrUnsupported)
	}
	frame, err := r3.NewFrame(origin, u, v)
	if err != nil {
		return sweepTransportFrame{}, fmt.Errorf(`%w: an arc span produced no transported frame: %s`, ErrUnsupported, err)
	}
	return newSweepTransportFrame(frame, originExact, uExact, vExact, nExact)
}

// exactSweepTransportFrame keeps quadrantal coordinate-axis transport exact.
// The rational rotation path above collapses to point intervals for these
// spans, so publishing those points directly avoids introducing libm's small
// nonzero cosine at a quarter turn. It is deliberately a strict fast path:
// any rounded coordinate or any normalization movement falls back to the
// bounded general transport.
func exactSweepTransportFrame(
	originExact, uExact, vExact, nExact ivVec3,
) (sweepTransportFrame, bool) {
	origin, okO := exactSweepIVVec(originExact)
	u, okU := exactSweepIVVec(uExact)
	v, okV := exactSweepIVVec(vExact)
	if !okO || !okU || !okV {
		return sweepTransportFrame{}, false
	}
	frame, err := r3.NewFrame(origin, u, v)
	if err != nil {
		return sweepTransportFrame{}, false
	}
	out, err := newSweepTransportFrame(frame, originExact, uExact, vExact, nExact)
	if err != nil || out.originBound != 0 || out.uBound != 0 || out.vBound != 0 || out.nBound != 0 {
		return sweepTransportFrame{}, false
	}
	return out, true
}

func exactSweepIVVec(vector ivVec3) (r3.Vec, bool) {
	components := [3]float64{}
	for i, coordinate := range vector {
		if coordinate.lo.Cmp(coordinate.hi) != 0 {
			return r3.Vec{}, false
		}
		held, exact := coordinate.lo.Float64()
		if !exact || math.IsNaN(held) || math.IsInf(held, 0) {
			return r3.Vec{}, false
		}
		components[i] = held
	}
	return r3.NewVec(components[0], components[1], components[2]), true
}

func newSweepTransportFrame(
	frame r3.Frame,
	originExact, uExact, vExact, nExact ivVec3,
) (sweepTransportFrame, error) {
	originBound, okO := sweepIVVecError(originExact, frame.Origin())
	uBound, okU := sweepIVVecError(uExact, frame.U())
	vBound, okV := sweepIVVecError(vExact, frame.V())
	nBound, okN := sweepIVVecError(nExact, frame.N())
	if !okO || !okU || !okV || !okN {
		return sweepTransportFrame{}, fmt.Errorf(`%w: a transported frame has no finite publication bound`, ErrUnsupported)
	}
	return sweepTransportFrame{
		frame: frame,

		originExact: originExact,
		uExact:      uExact,
		vExact:      vExact,
		nExact:      nExact,

		originBound: originBound,
		uBound:      uBound,
		vBound:      vBound,
		nBound:      nBound,
	}, nil
}

func sweepIVVecError(exact ivVec3, held r3.Vec) (float64, bool) {
	ex := intervalFloatError(exact[0], held.X)
	ey := intervalFloatError(exact[1], held.Y)
	ez := intervalFloatError(exact[2], held.Z)
	if !finiteAxisValues(ex, ey, ez) {
		return 0, false
	}
	squared := absSumUpper(productUpper(ex, ex), productUpper(ey, ey), productUpper(ez, ez))
	bound := upRound(math.Sqrt(squared))
	return bound, !math.IsNaN(bound) && !math.IsInf(bound, 0)
}

func sweepRatIntervalVec(vector sweepRatVec) ivVec3 {
	return ivVec3{
		pointInterval(vector[0]),
		pointInterval(vector[1]),
		pointInterval(vector[2]),
	}
}

func mustIVVec3Of(vector r3.Vec) ivVec3 {
	out, ok := ivVec3Of(vector)
	if !ok {
		panic("decad: a recorded Sweep path point must be finite")
	}
	return out
}

func heldSweepRatVec(vector sweepRatVec) (r3.Vec, bool) {
	x, _, okX := sweepRatHeld(vector[0])
	y, _, okY := sweepRatHeld(vector[1])
	z, _, okZ := sweepRatHeld(vector[2])
	return r3.NewVec(x, y, z), okX && okY && okZ
}

func heldSweepUnitVector(unit ivVec3) (r3.Vec, bool) {
	component := func(value ratInterval) float64 {
		midpoint := new(big.Rat).Add(value.lo, value.hi)
		midpoint.Quo(midpoint, big.NewRat(2, 1))
		held, _ := midpoint.Float64()
		return held
	}
	held := r3.NewVec(component(unit[0]), component(unit[1]), component(unit[2]))
	if !finiteVec(held) {
		return r3.Vec{}, false
	}
	return held.Normalize()
}
