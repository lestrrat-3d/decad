package twistsweep

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

type planeRecord = sectionrecord.PlaneRecord

// TwistAreaInterval integrates |dS/ds x dS/dt|. On an edge p(s)=p0+s*v,
// that norm is sqrt(h²|v|² + angle²(p(s)·v)²), independent of path station t.
// The primitive of sqrt(a+u²) is
// (u*sqrt(a+u²)+a*asinh(u/sqrt(a)))/2. All inputs and operations below are
// exact rational intervals, including the stated angle and asinh enclosure.
func TwistAreaInterval(poly TwistPolygon, height *big.Rat, angle proofbound.RatInterval) (proofbound.RatInterval, error) {
	result := proofbound.PointInterval(new(big.Rat).Mul(poly.Area, big.NewRat(2, 1)))
	absAngle := angle
	if angle.Hi.Sign() < 0 {
		absAngle = proofbound.IntervalNeg(angle)
	}
	if absAngle.Lo.Sign() <= 0 {
		return proofbound.RatInterval{}, fmt.Errorf(`%w: the twisted sweep's angle interval includes zero`, decaderr.ErrUnsupported)
	}
	h2 := new(big.Rat).Mul(height, height)
	for i, p := range poly.Points {
		q := poly.Points[(i+1)%len(poly.Points)]
		x, y := TwistRat(p.U), TwistRat(p.V)
		dx, dy := new(big.Rat).Sub(TwistRat(q.U), x), new(big.Rat).Sub(TwistRat(q.V), y)
		len2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
		base := new(big.Rat).Mul(h2, len2)
		rootA, ok := proofbound.SqrtInterval(proofbound.PointInterval(base))
		if !ok || rootA.Lo.Sign() <= 0 {
			return proofbound.RatInterval{}, fmt.Errorf(`%w: a twisted wall has no area bound`, decaderr.ErrUnsupported)
		}
		primitive := func(dot *big.Rat) (proofbound.RatInterval, bool) {
			u := proofbound.IntervalScale(absAngle, dot)
			root, ok := proofbound.SqrtInterval(proofbound.IntervalAdd(
				proofbound.PointInterval(base), proofbound.IntervalSquare(u)))
			if !ok {
				return proofbound.RatInterval{}, false
			}
			ratio, ok := proofbound.IntervalQuo(u, rootA)
			if !ok {
				return proofbound.RatInterval{}, false
			}
			return proofbound.IntervalScale(proofbound.IntervalAdd(
				proofbound.IntervalMul(u, root),
				proofbound.IntervalScale(proofbound.AsinhInterval(ratio), base),
			), big.NewRat(1, 2)), true
		}
		initial := new(big.Rat).Add(new(big.Rat).Mul(x, dx), new(big.Rat).Mul(y, dy))
		finish := new(big.Rat).Add(initial, len2)
		f0, ok0 := primitive(initial)
		f1, ok1 := primitive(finish)
		if !ok0 || !ok1 {
			return proofbound.RatInterval{}, fmt.Errorf(`%w: a twisted wall has no area integral`, decaderr.ErrUnsupported)
		}
		term, ok := proofbound.IntervalQuo(proofbound.IntervalSub(f1, f0),
			proofbound.IntervalScale(absAngle, len2))
		if !ok {
			return proofbound.RatInterval{}, fmt.Errorf(`%w: a twisted wall has no positive angle divisor`, decaderr.ErrUnsupported)
		}
		result = proofbound.IntervalAdd(result, term)
	}
	return result, nil
}

// TwistArcCells is the number of chord cells on the circular arm of a composite twist path.
const TwistArcCells = 8

// TwistCardinalPath is a straight run of Height followed by a quarter arc of Radius.
type TwistCardinalPath struct{ Height, Radius float64 }

// TwistStation is one placed section plane and frame of a twisted sweep, with its point error bound.
type TwistStation struct {
	Plane      planeRecord
	Frame      r3.Frame
	PointError float64
}

// CompositeTwistArea bounds the wall area of a twisted sweep over a straight run and a quarter arc.
func CompositeTwistArea(poly TwistPolygon, geometry TwistCardinalPath,
	joinAngle, fullAngle proofbound.RatInterval) (proofbound.RatInterval, error) {
	straight, err := TwistAreaInterval(poly, TwistRat(geometry.Height), joinAngle)
	if err != nil {
		return proofbound.RatInterval{}, err
	}
	h, radius, r := TwistRat(geometry.Height), TwistRat(geometry.Radius), TwistRat(poly.MaxRadiusL1)
	pathLength := proofbound.IntervalAdd(proofbound.PointInterval(h), proofbound.IntervalScale(proofbound.HalfPiIv, radius))
	arcRate, ok := proofbound.IntervalQuo(proofbound.IntervalScale(fullAngle, radius), pathLength)
	if !ok {
		return proofbound.RatInterval{}, fmt.Errorf(`%w: arc twist rate is unbounded`, decaderr.ErrUnsupported)
	}
	rate := proofbound.IntervalAbsUpper(arcRate)
	lower := new(big.Rat).Mul(TwistRat(poly.PerimeterLower),
		new(big.Rat).Mul(new(big.Rat).Sub(radius, r), proofbound.HalfPiIv.Lo))
	upperSpeed := new(big.Rat).Add(new(big.Rat).Add(radius, r), new(big.Rat).Mul(r, rate))
	upper := new(big.Rat).Mul(TwistRat(poly.PerimeterUpper),
		new(big.Rat).Mul(upperSpeed, proofbound.HalfPiIv.Hi))
	return proofbound.IntervalAdd(straight, proofbound.Interval(lower, upper)), nil
}

// CompositeTwistStations places the stations of a twisted sweep over a straight run and a quarter arc.
func CompositeTwistStations(theta proofbound.RatInterval, heldTheta float64,
	geometry TwistCardinalPath, poly TwistPolygon) ([]TwistStation, proofbound.RatInterval, error) {
	h, radius := TwistRat(geometry.Height), TwistRat(geometry.Radius)
	length := proofbound.IntervalAdd(proofbound.PointInterval(h), proofbound.IntervalScale(proofbound.HalfPiIv, radius))
	if length.Lo.Sign() <= 0 {
		return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twisted path length has no positive lower bound`, decaderr.ErrUnsupported)
	}
	joinFraction, ok := proofbound.IntervalQuo(proofbound.PointInterval(h), length)
	if !ok {
		return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twisted path join has no parameter bound`, decaderr.ErrUnsupported)
	}
	joinAngle := proofbound.IntervalMul(theta, joinFraction)
	stations := make([]TwistStation, TwistArcCells+2)
	arcLength := geometry.Radius * math.Pi / 2
	for index := range stations {
		var phiIv proofbound.RatInterval
		var phiHeld float64
		if index <= 1 {
			phiIv = proofbound.PointInterval(new(big.Rat))
		} else {
			fraction := big.NewRat(int64(index-1), TwistArcCells)
			phiIv = proofbound.IntervalScale(proofbound.HalfPiIv, fraction)
			phiHeld = math.Pi / 2 * float64(index-1) / TwistArcCells
		}
		pathFraction, ok := proofbound.IntervalQuo(proofbound.IntervalAdd(
			proofbound.PointInterval(h), proofbound.IntervalScale(phiIv, radius)), length)
		if !ok {
			return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twist station has no path fraction`, decaderr.ErrUnsupported)
		}
		betaIv := proofbound.IntervalMul(theta, pathFraction)
		betaHeld := heldTheta * (geometry.Height + geometry.Radius*phiHeld) / (geometry.Height + arcLength)
		if index == 0 {
			betaIv = proofbound.PointInterval(new(big.Rat))
			betaHeld = 0
		}
		sPhi, cPhi, okPhi := proofbound.RadSinCosSpan(phiIv)
		sBeta, cBeta, okBeta := proofbound.RadSinCosSpan(betaIv)
		if !okPhi || !okBeta {
			return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twist station has no trigonometric enclosure`, decaderr.ErrUnsupported)
		}
		sph, cph := math.Sincos(phiHeld)
		sbe, cbe := math.Sincos(betaHeld)
		origin := r3.NewVec(geometry.Radius*(1-cph), 0, geometry.Height+geometry.Radius*sph)
		if index == 0 {
			origin = r3.Vec{}
		}
		u := r3.NewVec(cph*cbe, sbe, -sph*cbe)
		v := r3.NewVec(-cph*sbe, cbe, sph*sbe)
		frame, err := r3.NewFrame(origin, u, v)
		if err != nil {
			return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twist station has no held frame: %s`, decaderr.ErrUnsupported, err)
		}
		if index == 0 {
			frame, err = r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
			if err != nil {
				return nil, proofbound.RatInterval{}, err
			}
		}
		trueOriginX := proofbound.IntervalScale(proofbound.IntervalSub(
			proofbound.PointInterval(big.NewRat(1, 1)), cPhi), radius)
		trueOriginZ := proofbound.IntervalAdd(proofbound.PointInterval(h), proofbound.IntervalScale(sPhi, radius))
		if index == 0 {
			trueOriginZ = proofbound.PointInterval(new(big.Rat))
		}
		originError := proofbound.AbsSumUpper(
			proofbound.IntervalFloatError(trueOriginX, frame.Origin().X),
			proofbound.IntervalFloatError(trueOriginZ, frame.Origin().Z))
		idealU := [3]proofbound.RatInterval{
			proofbound.IntervalMul(cPhi, cBeta), sBeta,
			proofbound.IntervalNeg(proofbound.IntervalMul(sPhi, cBeta)),
		}
		idealV := [3]proofbound.RatInterval{
			proofbound.IntervalNeg(proofbound.IntervalMul(cPhi, sBeta)), cBeta,
			proofbound.IntervalMul(sPhi, sBeta),
		}
		actualU, actualV := frame.U(), frame.V()
		frameError := 0.0
		for coordinate, actual := range [2][3]float64{{actualU.X, actualU.Y, actualU.Z},
			{actualV.X, actualV.Y, actualV.Z}} {
			ideal := idealU
			if coordinate == 1 {
				ideal = idealV
			}
			for axis, value := range actual {
				frameError = max(frameError, proofbound.IntervalFloatError(ideal[axis], value))
			}
		}
		pointError := proofbound.AbsSumUpper(originError,
			proofbound.ProductUpper(3*poly.MaxRadiusL1, frameError))
		if !proofbound.FiniteVec(frame.Origin()) || proofbound.IsNonFinite(pointError) {
			return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twist station has no finite displacement`, decaderr.ErrUnsupported)
		}
		stations[index] = TwistStation{
			Plane: planeRecord{Origin: frame.Origin(), U: frame.U(), V: frame.V()},
			Frame: frame, PointError: pointError,
		}
	}
	return stations, joinAngle, nil
}

// TwistWallCellAreaL1 bounds the INTEGRAL of the absolute area-density gap
// between a true rotating Line/Arc wall cell and its two held Loft triangles.
// Consequently it bounds the gap on every measurable subset selected by a
// later Boolean, even where positive and negative local errors cancel on the
// complete wall. The four corners are the actual held vertices in Loft's
// (vLo,vHi,wLo,wHi) order. Their source stations are each within vertexDelta.
//
// Let s run along the source cell and u run between the two twist stations.
// The held triangles cover u<=s and s<=u in the unit square. If T is the
// true rotating wall and H is the affine triangle on either half, then
// |area(T)-area(H)| <= |T_s-H_s|*|T_u| + |H_s|*|T_u-H_u| pointwise.
//
// The integral of |T_s-H_s| is at most I + L*a²/8 + 2e + W/3. Here I is
// the authenticated tangent-deviation integral (bounded by sqrt(energy)),
// L is source speed, a is cell turn, e is endpoint displacement, and W is
// the held quad twist vector. The a²/8 term is the linear-interpolation
// remainder for rotation as a function of u. Integrating the affine-versus-
// bilinear triangle split gives W/3 over both halves.
//
// Likewise, integral |T_u-H_u| <= R*a²/2 + a*matched + 2e + W/3.
// The first term bounds derivative-versus-secant rotation; the second uses
// |R1-R0|<=a on the parameter-matched curve/chord departure. The remaining
// terms are the same endpoint and diagonal-split obligations. |T_u| is at
// most height+a*R, while |H_s| is at most the larger held section chord.
// All scalars below are rounded outward; invalid caller claims return +Inf.
func TwistWallCellAreaL1(vLo, vHi, wLo, wHi r3.Vec,
	claim loftrecord.TwistAreaCellClaims, vertexDelta, radius, angle, height float64) float64 {
	for _, x := range []float64{claim.ArcLength, claim.Matched, vertexDelta, radius, angle, height} {
		if proofbound.IsNonFinite(x) || x < 0 {
			return math.Inf(1)
		}
	}
	if math.IsNaN(claim.Energy) || claim.Energy < 0 ||
		!proofbound.FiniteVec(vLo) || !proofbound.FiniteVec(vHi) ||
		!proofbound.FiniteVec(wLo) || !proofbound.FiniteVec(wHi) {
		return math.Inf(1)
	}
	dv := proofbound.HeldDelta(vHi, vLo)
	dw := proofbound.HeldDelta(wHi, wLo)
	chordV, chordW := proofbound.DvLenUpper(dv), proofbound.DvLenUpper(dw)
	maxChord := math.Max(chordV, chordW)
	// A held chord may differ from its exact source chord by two endpoint
	// displacements. The source arc-length claim must still reach it.
	if !proofbound.DvLenAtLeast(proofbound.AbsSumUpper(claim.ArcLength,
		proofbound.ProductUpper(2, vertexDelta)), dv) ||
		!proofbound.DvLenAtLeast(proofbound.AbsSumUpper(claim.ArcLength,
			proofbound.ProductUpper(2, vertexDelta)), dw) {
		return math.Inf(1)
	}
	twist := proofbound.DvLenUpper(proofarith.DvSub(dw, dv))
	freeI := proofbound.AbsSumUpper(claim.ArcLength, maxChord,
		proofbound.ProductUpper(2, vertexDelta))
	i := freeI
	if !proofbound.IsNonFinite(claim.Energy) {
		i = math.Min(freeI, proofbound.UpRound(math.Sqrt(claim.Energy)))
	}
	a2 := proofbound.ProductUpper(angle, angle)
	twoE := proofbound.ProductUpper(2, vertexDelta)
	warp := proofbound.DivUpper(twist, 3)
	sGap := proofbound.AbsSumUpper(i,
		proofbound.DivUpper(proofbound.ProductUpper(claim.ArcLength, a2), 8), twoE, warp)
	uGap := proofbound.AbsSumUpper(
		proofbound.DivUpper(proofbound.ProductUpper(radius, a2), 2),
		proofbound.ProductUpper(angle, claim.Matched), twoE, warp)
	uSpeed := proofbound.AbsSumUpper(height, proofbound.ProductUpper(angle, radius))
	return proofbound.AbsSumUpper(proofbound.ProductUpper(uSpeed, sGap),
		proofbound.ProductUpper(maxChord, uGap))
}
