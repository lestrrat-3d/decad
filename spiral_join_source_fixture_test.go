package decad

import (
	"math"
	"slices"
	"sort"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixture below builds the source-default 31/31 module-1 spiral bevel
// member: a revolved six-sided blank, and one fitted tooth sliced into eight
// slabs, twisted, crowned, lofted through nine sections and trimmed by the toe
// and heel cones. It is the smallest geometry that satisfies the spiral
// first-join certificate (nine sections, two fitted flanks of fourteen spans).
// The numbers come from a 35 degree right-hand spiral on a 90 degree shaft.

const (
	spiralModule        = 1.0
	spiralTeeth         = 31
	spiralSamples       = 15
	spiralPressureAngle = 20 * math.Pi / 180
	spiralAngle         = 35 * math.Pi / 180
	spiralCrownPerRad   = 0.5
)

type spiralAxial struct{ station, radius float64 }

type spiralDims struct {
	coneAngle, coneDist, width float64
}

func spiralDerive() spiralDims {
	r := spiralModule * spiralTeeth / 2
	gamma := math.Atan2(math.Sin(math.Pi/2)*r, r+math.Cos(math.Pi/2)*r)
	coneDist := r / math.Sin(gamma)
	maxWidth := 0.95 * coneDist * math.Pow(math.Sin(gamma), 2)
	width := math.Min(math.Hypot(2*r, 2*r)/6, maxWidth)
	return spiralDims{coneAngle: gamma, coneDist: coneDist, width: width}
}

// spiralBlankSection is the six-point axial section of the finite blank.
func spiralBlankSection(d spiralDims) []spiralAxial {
	r := spiralModule * spiralTeeth / 2
	dedendum := 1.25 * spiralModule
	baseHeight := math.Min(math.Max(spiralModule*spiralTeeth/8,
		1.05*dedendum*math.Sin(d.coneAngle)),
		0.95*(r-dedendum*math.Cos(d.coneAngle))*math.Tan(d.coneAngle))
	heelStation := d.coneDist*math.Cos(d.coneAngle) + baseHeight
	heelRadius := r - baseHeight/math.Tan(d.coneAngle)
	ded := spiralAxial{
		station: d.coneDist*math.Cos(d.coneAngle) + dedendum*math.Sin(d.coneAngle),
		radius:  r - dedendum*math.Cos(d.coneAngle),
	}
	toeRadius := r - d.width/math.Sin(d.coneAngle)
	apexToDed := math.Hypot(d.coneDist, dedendum)
	rootLength := d.width * apexToDed / d.coneDist
	toeFactor := 1 - rootLength/apexToDed
	toeRoot := spiralAxial{station: toeFactor * ded.station, radius: toeFactor * ded.radius}
	toeSlide := (toeRoot.radius - toeRadius) / math.Cos(d.coneAngle)
	toeStation := toeRoot.station + toeSlide*math.Sin(d.coneAngle)
	return []spiralAxial{
		{station: toeStation, radius: 0},
		{station: heelStation, radius: 0},
		{station: heelStation, radius: heelRadius},
		ded,
		toeRoot,
		{station: toeStation, radius: toeRadius},
	}
}

func spiralRevolve(t *testing.T, doc *Document, section []spiralAxial) *Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(section))
	for i, xy := range section {
		points[i] = s.CreatePoint(xy.station, xy.radius)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	require.True(t, s.Profiles()[0].Valid)
	body, err := Revolve(t.Context(), doc, s, s.Profiles()[0],
		SketchLine{Start: Point2{}, End: Point2{U: 1}}, FullRevolution{})
	require.NoError(t, err)
	return body
}

// spiralInvolute samples the left and right flanks of one virtual tooth.
func spiralInvolute(pitch, base, tip, effectiveTeeth float64) ([][2]float64, [][2]float64) {
	involute := func(radius float64) [2]float64 {
		tan := math.Tan(math.Acos(base / radius))
		return [2]float64{
			base * (math.Cos(tan) + tan*math.Sin(tan)),
			base * (math.Sin(tan) - tan*math.Cos(tan)),
		}
	}
	rotate := func(p [2]float64, angle float64) [2]float64 {
		sine, cosine := math.Sin(angle), math.Cos(angle)
		return [2]float64{p[0]*cosine - p[1]*sine, p[0]*sine + p[1]*cosine}
	}
	atPitch := involute(pitch)
	rotation := math.Pi/(2*effectiveTeeth) - math.Atan2(-atPitch[1], atPitch[0])
	left := make([][2]float64, spiralSamples)
	right := make([][2]float64, spiralSamples)
	for i := range left {
		sample := involute(base + (tip-base)*float64(i)/float64(spiralSamples-1))
		sample[1] = -sample[1]
		sample = rotate(sample, rotation)
		left[i] = rotate(sample, math.Pi)
		sample[1] = -sample[1]
		right[i] = rotate(sample, math.Pi)
	}
	return left, right
}

// spiralPointTooth lofts one fitted back-cone tooth from the common apex.
// The tangent witnesses pass zero tilt to keep the original back-cone plane.
func spiralPointTooth(t *testing.T, doc *Document, d spiralDims, tilt float64) *Body {
	t.Helper()
	world := sketch.NewWorld()
	center := r3.NewVec(d.coneDist/math.Cos(d.coneAngle), 0, 0)
	tilted := d.coneAngle + tilt
	frame, err := r3.NewFrame(center, r3.NewVec(math.Sin(tilted), -math.Cos(tilted), 0),
		r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)

	pitch := spiralModule * spiralTeeth / (2 * math.Cos(d.coneAngle))
	base := pitch * math.Cos(spiralPressureAngle)
	tip := pitch + spiralModule
	root := pitch - (1.25+0.1125)*spiralModule
	require.Greater(t, base, root, "the fixture tooth has a straight root, not an embedded one")
	left, right := spiralInvolute(pitch, base, tip, spiralTeeth/math.Cos(d.coneAngle))

	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	fixed := func(xy [2]float64) *sketch.Point {
		point := s.CreatePoint(xy[0], xy[1])
		s.Fix(point)
		return point
	}
	leftPoints := make([]*sketch.Point, spiralSamples)
	rightPoints := make([]*sketch.Point, spiralSamples)
	for i := range left {
		leftPoints[i] = fixed(left[i])
		rightPoints[i] = fixed(right[i])
	}
	_, err = s.CreateFitSpline(rightPoints...)
	require.NoError(t, err)
	s.CreateArc(origin, rightPoints[spiralSamples-1], leftPoints[spiralSamples-1])
	_, err = s.CreateFitSpline(leftPoints...)
	require.NoError(t, err)
	rightRoot := fixed([2]float64{right[0][0] * root / base, right[0][1] * root / base})
	leftRoot := fixed([2]float64{left[0][0] * root / base, left[0][1] * root / base})
	s.CreateLine(rightRoot, rightPoints[0])
	s.CreateLine(leftPoints[0], leftRoot)
	s.CreateArc(origin, rightRoot, leftRoot)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	require.True(t, s.Profiles()[0].Valid)
	require.Len(t, s.Profiles()[0].Outer, 6)
	tooth, err := LoftFromPoint(t.Context(), doc, r3.Vec{}, s, s.Profiles()[0])
	require.NoError(t, err)
	return tooth
}

// spiralFrame holds the root-cone distances that place the spiral steps.
type spiralFrame struct {
	coneVec    r3.Vec
	toe, heel  float64
	mean, span float64
}

func spiralSlabFrame(section []spiralAxial) spiralFrame {
	to3 := func(p spiralAxial) r3.Vec { return r3.NewVec(p.station, p.radius, 0) }
	toeCone, heelCone := to3(section[4]), to3(section[3])
	toeMid := toeCone.Add(to3(section[5])).Scale(0.5)
	heelMid := heelCone.Add(to3(section[2])).Scale(0.5)
	coneVec := heelCone.Scale(1 / heelCone.Len())
	f := spiralFrame{coneVec: coneVec, toe: toeMid.Dot(coneVec), heel: heelMid.Dot(coneVec)}
	f.mean = (f.toe + f.heel) / 2
	f.span = f.heel - f.toe
	return f
}

func spiralSliceSheet(t *testing.T, doc *Document, origin, along r3.Vec, extent float64) *Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	a, b := origin.Add(along.Scale(-extent)), origin.Add(along.Scale(extent))
	first, second := s.CreatePoint(a.X, a.Y), s.CreatePoint(b.X, b.Y)
	s.Fix(first)
	s.Fix(second)
	s.CreateLine(first, second)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	sheet, err := Extrude(t.Context(), doc, s, s.Chains()[0], TwoSided{
		One: DistanceSide{D: units.Millimeters(extent)},
		Two: DistanceSide{D: units.Millimeters(extent)},
	})
	require.NoError(t, err)
	return sheet
}

// spiralSlabs cuts the point tooth at eight parent-plane-parallel planes and
// returns the seven or eight heel-side slabs, toe to heel, without the apex scrap.
func spiralSlabs(t *testing.T, doc *Document, d spiralDims, f spiralFrame,
	tooth *Body) []*Body {
	t.Helper()
	u := r3.NewVec(math.Sin(d.coneAngle), -math.Cos(d.coneAngle), 0)
	normal := u.Cross(r3.NewVec(0, 0, 1))
	parent := r3.NewVec(d.coneDist/math.Cos(d.coneAngle), 0, 0)
	sign := 1.0
	if parent.Scale(-1).Dot(normal) < 0 {
		sign = -1
	}
	pieces := []*Body{tooth}
	for k := 1; k <= 8; k++ {
		location := parent.Add(normal.Scale(sign * float64(k) * f.span / 6))
		sheet := spiralSliceSheet(t, doc, location, u, 2*d.coneDist)
		cut, err := Split(t.Context(), pieces[0], sheet)
		require.NoError(t, err)
		require.Len(t, cut, 2)
		pieces[0] = cut[0]
		pieces = append(pieces, cut[1])
	}
	type ranked struct {
		body     *Body
		distance float64
	}
	order := make([]ranked, len(pieces))
	for i, piece := range pieces {
		centroid, err := piece.Centroid()
		require.NoError(t, err)
		order[i] = ranked{body: piece, distance: centroid.Value.Dot(f.coneVec)}
	}
	sort.Slice(order, func(i, j int) bool { return order[i].distance < order[j].distance })
	require.NoError(t, Remove(order[0].body))
	slabs := make([]*Body, 0, len(order)-1)
	for _, o := range order[1:] {
		slabs = append(slabs, o.body)
	}
	return slabs
}

// spiralTrace returns the total crown roll of the cutter-circle arc.
func spiralTraceTotal(t *testing.T, d spiralDims, f spiralFrame) float64 {
	t.Helper()
	cutter := f.mean
	cx := f.mean - cutter*math.Sin(spiralAngle)
	cy := cutter * math.Cos(spiralAngle)
	near := func(radius float64) [2]float64 {
		centerDistance := math.Hypot(cx, cy)
		along := (radius*radius - cutter*cutter + centerDistance*centerDistance) / (2 * centerDistance)
		height := math.Sqrt(math.Max(0, radius*radius-along*along))
		ux, uy := cx/centerDistance, cy/centerDistance
		first := [2]float64{along*ux - height*uy, along*uy + height*ux}
		second := [2]float64{along*ux + height*uy, along*uy - height*ux}
		if math.Hypot(first[0]-f.mean, first[1]) <= math.Hypot(second[0]-f.mean, second[1]) {
			return first
		}
		return second
	}
	toe := near(f.toe - 0.06*f.span)
	heel := near(f.heel + 0.06*f.span)
	phi := math.Atan2(heel[1], heel[0]) - math.Atan2(toe[1], toe[0])
	return math.Abs(phi) / math.Sin(d.coneAngle)
}

func spiralHeelFace(t *testing.T, slab *Body, f spiralFrame) (*Face, float64) {
	t.Helper()
	_, heel, err := slab.ExtremeFacesByCentroid(t.Context(), f.coneVec)
	require.NoError(t, err)
	center, err := heel.Centroid()
	require.NoError(t, err)
	return heel, center.Value.Dot(f.coneVec)
}

// spiralTwistSlabs applies the heel-keyed shaft rotation to every slab. It also
// returns the turn applied to the heel-most slab, the one with the largest
// pre-twist heel distance.
func spiralTwistSlabs(t *testing.T, slabs []*Body, f spiralFrame,
	total float64) ([]*Body, r3.Transform) {
	t.Helper()
	shaft := r3.NewVec(1, 0, 0)
	out := make([]*Body, 0, len(slabs))
	var heelTurn r3.Transform
	heelDistance := math.Inf(-1)
	for _, slab := range slabs {
		_, distance := spiralHeelFace(t, slab, f)
		angle := -total * (f.mean - distance) / f.span
		turn, err := r3.RotationAround(r3.Vec{}, shaft, units.Radians(angle))
		require.NoError(t, err)
		if distance > heelDistance {
			heelDistance, heelTurn = distance, turn
		}
		placed, err := Transform(t.Context(), slab, turn)
		require.NoError(t, err)
		out = append(out, placed)
	}
	return out, heelTurn
}

// spiralRootAnchor is the midpoint of the two root corners of a heel face.
func spiralRootAnchor(t *testing.T, heel *Face) r3.Vec {
	t.Helper()
	type radial struct {
		point  r3.Vec
		radius float64
		bound  float64
	}
	seen := make(map[*Vertex]struct{})
	var vertices []radial
	for _, edge := range heel.Edges() {
		for _, vertex := range []*Vertex{edge.Start(), edge.End()} {
			if _, ok := seen[vertex]; ok {
				continue
			}
			seen[vertex] = struct{}{}
			reading := vertex.Position()
			vertices = append(vertices, radial{point: reading.Value,
				radius: math.Hypot(reading.Value.Y, reading.Value.Z), bound: reading.Bound.Base()})
		}
	}
	require.GreaterOrEqual(t, len(vertices), 3)
	sort.Slice(vertices, func(i, j int) bool { return vertices[i].radius < vertices[j].radius })
	rootUpper := math.Max(vertices[0].radius+vertices[0].bound, vertices[1].radius+vertices[1].bound)
	otherLower := math.Inf(1)
	for _, v := range vertices[2:] {
		otherLower = math.Min(otherLower, v.radius-v.bound)
	}
	require.Less(t, rootUpper, otherLower, "the two root corners are distinguishable by position bounds")
	return vertices[0].point.Add(vertices[1].point).Scale(0.5)
}

// spiralCrownSlabs scales every slab except the heel-most one about its heel root.
func spiralCrownSlabs(t *testing.T, slabs []*Body, f spiralFrame, total float64) []*Body {
	t.Helper()
	heels := make([]*Face, len(slabs))
	distances := make([]float64, len(slabs))
	heelMost := 0
	for i, slab := range slabs {
		heels[i], distances[i] = spiralHeelFace(t, slab, f)
		if distances[i] > distances[heelMost] {
			heelMost = i
		}
	}
	out := make([]*Body, len(slabs))
	for i, slab := range slabs {
		if i == heelMost {
			out[i] = slab
			continue
		}
		u := (f.heel - distances[i]) / f.span
		factor := 1 - spiralCrownPerRad*(math.Abs(total)/2)*u
		require.Positive(t, factor)
		scaled, err := ScaledAt(t.Context(), slab, spiralRootAnchor(t, heels[i]), units.Scalar(factor))
		require.NoError(t, err)
		out[i] = scaled
	}
	return out
}

// spiralLoftSections picks the nine sections: the first slab's toe face, then
// every slab's heel face, ordered by distance along the root cone.
func spiralLoftSections(t *testing.T, slabs []*Body, f spiralFrame) []*Face {
	t.Helper()
	type section struct {
		toe, heel *Face
		distance  float64
	}
	sections := make([]section, len(slabs))
	for i, slab := range slabs {
		toe, heel, err := slab.ExtremeFacesByCentroid(t.Context(), f.coneVec)
		require.NoError(t, err)
		center, err := heel.Centroid()
		require.NoError(t, err)
		sections[i] = section{toe: toe, heel: heel, distance: center.Value.Dot(f.coneVec)}
	}
	slices.SortFunc(sections, func(a, b section) int {
		if a.distance < b.distance {
			return -1
		}
		if a.distance > b.distance {
			return 1
		}
		return 0
	})
	faces := []*Face{sections[0].toe}
	for _, s := range sections {
		faces = append(faces, s.heel)
	}
	return faces
}

// spiralBlankConeRef names the blank's cone face whose meridian runs from p0
// to p1. The expected apex is x0 + r0*(x1-x0)/(r0-r1); the face is the unique
// Cone-tagged one whose Origin.X lies within 1e-9 mm of it. This chooses
// intent through a public read and proves nothing.
func spiralBlankConeRef(t *testing.T, blank *Body, p0, p1 spiralAxial) FeatureRef {
	t.Helper()
	apex := p0.station + p0.radius*(p1.station-p0.station)/(p0.radius-p1.radius)
	var found []*Face
	for _, face := range blank.Faces() {
		cone, ok := face.Surface().(Cone)
		if ok && math.Abs(cone.Origin.X-apex) <= 1e-9 {
			found = append(found, face)
		}
	}
	require.Len(t, found, 1, "exactly one blank cone face has the expected apex")
	require.NotEmpty(t, found[0].Origins())
	return found[0].Origins()[0]
}

// spiralTangentBlankAndTooth builds the L3 witness through Revolve, Cut,
// LoftFacesToConeTangent, and Intersect. Neither body has been combined yet.
func spiralTangentBlankAndTooth(t *testing.T, doc *Document) (blank, trimmed, curved *Body) {
	return spiralTangentBlankAndToothWithTools(t, doc, 0, true)
}

func spiralTangentBlankAndToothWithTools(t *testing.T, doc *Document, toeShift float64,
	heelWitness bool) (blank, trimmed, curved *Body) {
	t.Helper()
	d := spiralDerive()
	section := spiralBlankSection(d)
	blank = spiralRevolve(t, doc, section)

	far := units.Millimeters(-0.1 * d.coneDist)
	coneOf := func(a, b spiralAxial) *Body {
		ref := spiralBlankConeRef(t, blank, a, b)
		cone, err := ConeFromFace(t.Context(), blank, Faces(FaceCreatedBy(ref)).Exactly(1), far)
		require.NoError(t, err)
		return cone
	}
	var heel *Body
	if heelWitness {
		// Move the heel apex one float step inside the blank cone while keeping
		// its far radius two steps outside. The finite heel gap still passes.
		heelApex := math.Nextafter(section[3].station+section[3].radius, math.Inf(-1))
		heelFar := -0.1 * d.coneDist
		heelRadius := heelApex - heelFar
		heelRadius = math.Nextafter(math.Nextafter(heelRadius, math.Inf(1)), math.Inf(1))
		heel = spiralRevolve(t, doc, []spiralAxial{
			{station: heelFar}, {station: heelFar, radius: heelRadius}, {station: heelApex},
		})
	} else {
		heel = coneOf(section[3], section[2])
	}
	var toe *Body
	if toeShift != 0 {
		apex := section[4].station + section[4].radius + toeShift
		farStation := -0.1 * d.coneDist
		toe = spiralRevolve(t, doc, []spiralAxial{
			{station: farStation}, {station: farStation, radius: apex - farStation}, {station: apex},
		})
	} else {
		toe = coneOf(section[4], section[5])
	}

	point := spiralPointTooth(t, doc, d, 0)
	f := spiralSlabFrame(section)
	slabs := spiralSlabs(t, doc, d, f, point)
	total := spiralTraceTotal(t, d, f)
	twisted, heelTurn := spiralTwistSlabs(t, slabs, f, total)
	crowned := spiralCrownSlabs(t, twisted, f, total)
	faces := spiralLoftSections(t, crowned, f)

	// The heel-most selected cap is the one the constructed tangent section
	// replaces. It must belong to the heel-most slab, whose turn gives the
	// tangent ruling's radial direction.
	dropped := faces[len(faces)-1]
	require.Contains(t, crowned[len(crowned)-1].Faces(), dropped,
		"the dropped cap belongs to the heel-most slab")
	var err error
	curved, err = LoftFacesToConeTangent(t.Context(),
		ConeTangentEnd{Cone: heel, Radial: heelTurn.ApplyDir(r3.NewVec(0, 1, 0))},
		faces[:len(faces)-1]...)
	require.NoError(t, err)
	for _, slab := range crowned {
		require.NoError(t, Remove(slab))
	}

	toeKeeper, err := Cut(t.Context(), curved, toe)
	require.NoError(t, err)
	trimmed, err = Intersect(t.Context(), toeKeeper, heel)
	require.NoError(t, err)
	return blank, trimmed, curved
}
