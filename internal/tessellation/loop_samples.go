package tessellation

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ChordSamples is one boundary loop's chording. Sample j belongs to FaceOf[j]
// and has the outgoing walk's sagitta and its own recorded-point enclosure.
// WallSlack and CapSlack keep the two area charges separate so a sheet can
// omit the cap charge. SegmentArea is also read by the volume proof.
// PerimeterUpper bounds the analytic loop length used in a section
// displacement's area charge.
type ChordSamples[F any] struct {
	Samples        []sectionrecord.Point2
	FaceOf         []F
	SagOf          []float64
	BoundOf        []proofbound.WalkEndBound
	MaxSag         float64
	WallSlack      float64
	CapSlack       float64
	SegmentArea    float64
	Walks          int
	PerimeterUpper float64
}

// SampleLoop emits the starts and interior stations of already coalesced
// walks, charging budget before each walk and each emitted station. The caller
// supplies the recorded segments, the source face for each walk, and the
// recorded-point enclosure of an interior circular station. The shared work
// counter is passed through to free-form chording.
//
// Walk i's start is the sample it shares with walk i−1, the chord of both
// ending there, so its bound reaches the points both neighbours' records
// denote (boundarywalk.JunctionStartBound): an arc's natural t = 1 end adds its
// radial residual, and a cut junction the gap between the two held ends. A
// lone walk that does not close, a brep face's open wall, meets no neighbour
// here: its start reaches the point its own record denotes
// (boundarywalk.DenotedStartBound), and the caller charges every other use
// meeting there.
func SampleLoop[F any](walks []survey2d.SideWalk, segments []sectionrecord.CurveSegment,
	chord, height float64, work *freeform.FreeformWork, budget *proofbound.WorkBudget,
	wallFace func(survey2d.SideWalk) (F, error),
	stationBound func(sectionrecord.CurveSegment, int, int, float64, float64) proofbound.WalkEndBound,
) (ChordSamples[F], error) {
	var samples []sectionrecord.Point2
	var faceOf []F
	var sagOf []float64
	var boundOf []proofbound.WalkEndBound
	var maxSag, wallSlack, capSlack, segmentArea float64
	for i, w := range walks {
		if err := budget.Step(); err != nil {
			return ChordSamples[F]{}, err
		}
		face, err := wallFace(w)
		if err != nil {
			return ChordSamples[F]{}, err
		}
		start := sampleStartBound(walks, segments, i)
		// A switch on survey2d.WalkKind must be total: a straight walk
		// contributes its start, a free-form walk its Bézier stations,
		// and a circular walk its angular stations.
		switch w.Kind {
		case survey2d.WalkLine:
			samples = append(samples, sectionrecord.Point2{U: w.StartU, V: w.StartV})
			faceOf = append(faceOf, face)
			sagOf = append(sagOf, 0)
			boundOf = append(boundOf, start)
		case survey2d.WalkFreeform:
			chain, err := freeform.ChainStations(w.Spans, chord, work)
			if err != nil {
				return ChordSamples[F]{}, err
			}
			pts, bounds, err := freeformWalkStations(w, chain, start)
			if err != nil {
				return ChordSamples[F]{}, err
			}
			for i, p := range pts {
				if err := budget.Step(); err != nil {
					return ChordSamples[F]{}, err
				}
				samples = append(samples, p)
				faceOf = append(faceOf, face)
				sagOf = append(sagOf, chain.Sagitta)
				boundOf = append(boundOf, bounds[i])
			}
			maxSag = math.Max(maxSag, chain.Sagitta)
			wall, segment := freeformChordAreas(chain, height)
			wallSlack = proofbound.AbsSumUpper(wallSlack, wall)
			capSlack = proofbound.AbsSumUpper(capSlack, segment)
			segmentArea = proofbound.AbsSumUpper(segmentArea, segment)
		case survey2d.WalkCircular:
			n, sag, err := ChordCount(w.SegmentWalk, chord, ChordWalkMin(w.SegmentWalk))
			if err != nil {
				return ChordSamples[F]{}, err
			}
			maxSag = math.Max(maxSag, sag)
			wallSlack = proofbound.AbsSumUpper(wallSlack, proofbound.ProductUpper(WalkWallSlack(w.SegmentWalk, n, height), 1+1e-9))
			capSlack = proofbound.AbsSumUpper(capSlack, proofbound.ProductUpper(WalkSegmentArea(w.SegmentWalk, n), 1+1e-9))
			segmentArea = proofbound.AbsSumUpper(segmentArea, WalkSegmentArea(w.SegmentWalk, n))
			// Circular walks are not coalesced. Their stations name exact
			// fractions of the recorded segment's parameter window.
			seg := segments[w.Segs[0]]
			dth := (w.Th1 - w.Th0) / float64(n)
			for k := range n {
				if err := budget.Step(); err != nil {
					return ChordSamples[F]{}, err
				}
				p := sectionrecord.Point2{U: w.StartU, V: w.StartV}
				bound := start
				if k > 0 {
					th := w.Th0 + float64(k)*dth
					p = sectionrecord.Point2{U: w.CU + w.Radius*math.Cos(th), V: w.CV + w.Radius*math.Sin(th)}
					bound = stationBound(seg, k, n, p.U, p.V)
				}
				samples = append(samples, p)
				faceOf = append(faceOf, face)
				sagOf = append(sagOf, sag)
				boundOf = append(boundOf, bound)
			}
		default:
			return ChordSamples[F]{}, fmt.Errorf(`%w: chording a boundary loop does not support walk kind %d`, decaderr.ErrUnsupported, w.Kind)
		}
	}
	return ChordSamples[F]{
		Samples:     samples,
		FaceOf:      faceOf,
		SagOf:       sagOf,
		BoundOf:     boundOf,
		MaxSag:      maxSag,
		WallSlack:   wallSlack,
		CapSlack:    capSlack,
		SegmentArea: segmentArea,
		Walks:       len(walks),
	}, nil
}

// sampleStartBound is the bound SampleLoop charges walk i's start: the
// junction bound against walk i−1's end around a loop, or the walk's own
// denoted start for a lone open walk.
func sampleStartBound(walks []survey2d.SideWalk, segments []sectionrecord.CurveSegment, i int) proofbound.WalkEndBound {
	w := walks[i]
	next := segments[w.Segs[0]]
	if len(walks) == 1 && !w.Closed {
		return boundarywalk.DenotedStartBound(next, w.SegmentWalk)
	}
	prev := walks[(i+len(walks)-1)%len(walks)]
	return boundarywalk.JunctionStartBound(segments[prev.Segs[len(prev.Segs)-1]], prev.SegmentWalk, next, w.SegmentWalk)
}

// freeformWalkStations emits one exact chain station per chord, including
// the walk's start and excluding its end, which the next walk emits. A
// reversed walk starts at the chain end and then visits the stations in
// reverse order. The shared junction uses the walk's held start verbatim,
// charged start (SampleLoop's junction bound). Each interior station is
// rounded once and charged its own rational gap.
func freeformWalkStations(w survey2d.SideWalk, chain freeform.FreeformChain, start proofbound.WalkEndBound) ([]sectionrecord.Point2, []proofbound.WalkEndBound, error) {
	if len(chain.Stations) == 0 {
		return nil, nil, fmt.Errorf(`%w: a free-form walk chorded to no station has no boundary sample`, decaderr.ErrDegenerate)
	}
	ordered := make([]freeform.RatPoint, 0, len(chain.Stations))
	if !w.Reversed {
		ordered = append(ordered, chain.Stations...)
	} else {
		ordered = append(ordered, chain.End)
		for i := len(chain.Stations) - 1; i >= 1; i-- {
			ordered = append(ordered, chain.Stations[i])
		}
	}
	pts := make([]sectionrecord.Point2, len(ordered))
	bounds := make([]proofbound.WalkEndBound, len(ordered))
	for i, station := range ordered {
		p, ok := splinebezier.Point2Of(station)
		if !ok {
			return nil, nil, fmt.Errorf(`%w: a free-form chord station has no representable plane coordinate`, decaderr.ErrUnsupported)
		}
		bound := proofbound.WalkEndBound{
			U: proofarith.RationalFloatError(station.U, p.U),
			V: proofarith.RationalFloatError(station.V, p.V),
		}
		if !bound.Derivable() {
			return nil, nil, fmt.Errorf(`%w: a free-form chord station states no bound on the rounding its held plane coordinates commit`, decaderr.ErrUnsupported)
		}
		pts[i], bounds[i] = p, bound
	}
	if !w.StartBound.Derivable() {
		return nil, nil, fmt.Errorf(`%w: a free-form walk states no bound on its own start, so the junction it shares carries no displacement`, decaderr.ErrUnsupported)
	}
	pts[0] = sectionrecord.Point2{U: w.StartU, V: w.StartV}
	bounds[0] = start
	return pts, bounds, nil
}

// freeformChordAreas sums each cell's wall arc-minus-chord allowance and
// planar two-sided sagitta tube. The caller charges the planar term once
// per cap and once against sweep height for occupied volume.
func freeformChordAreas(chain freeform.FreeformChain, height float64) (float64, float64) {
	wall, segment := 0.0, 0.0
	h := math.Abs(height)
	for k, arc := range chain.CellArcUpper {
		deficit := proofbound.UpRound(math.Max(arc-chain.CellChordLower[k], 0))
		wall = proofbound.AbsSumUpper(wall, proofbound.ProductUpper(deficit, h))
		segment = proofbound.AbsSumUpper(segment, proofbound.SectionDisplacementArea(chain.Sagitta, 1, arc))
	}
	return wall, segment
}
