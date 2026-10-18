package clearance

import (
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// BodyCarriers contains the analytic faces and their displacement charges.
// The caller still owns body admission, topology, and result publication.
type BodyCarriers struct {
	Faces []*CFace
	Delta float64
}

// BuildPrismBodyCarriers reads the same live prism record used to build its
// topology. A displaced section cannot certify an exact carrier model.
func BuildPrismBodyCarriers(budget *proofbound.WorkBudget, pp prismrecord.Payload) (BodyCarriers, bool, error) {
	var no BodyCarriers
	if pp.SectionDelta != 0 {
		return no, false, nil
	}
	loops, err := boundarywalk.SurveyLoops(budget, boundarywalk.Profile(pp.Profile))
	if err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return no, false, err
		}
		return no, false, nil
	}
	built, ok, err := BuildPrismCarriers(budget, prismCarrierFrame(pp), loops, pp.SurfaceResult)
	if err != nil || !ok {
		return no, false, err
	}
	pointTerm := CarrierLiftRound(built.Faces)
	delta := proofbound.AbsSumUpper(pointTerm, pp.AxialDelta(),
		BodyFaceTiltDelta(built.Faces, pp.Frame, pp.Xform))
	return BodyCarriers{Faces: built.Faces, Delta: delta}, true, nil
}

// CarrierLiftRound returns the largest frame or placement lift rounding
// recorded by the faces.
func CarrierLiftRound(faces []*CFace) float64 {
	round := 0.0
	for _, f := range faces {
		round = math.Max(round, f.LiftRound)
	}
	return round
}

func prismCarrierFrame(pp prismrecord.Payload) PrismCarrierFrame {
	return PrismCarrierFrame{Frame: pp.Frame, Transform: pp.Xform, Z0: pp.Z0, Z1: pp.Z1}
}

// BuildBrepBodyCarriers reads the record's own face order and refuses route-L
// bands and displaced sections before it builds any face carrier.
func BuildBrepBodyCarriers(budget *proofbound.WorkBudget, bp breprecord.Payload) (BodyCarriers, bool, error) {
	var no BodyCarriers
	if bp.SectionDelta() != 0 || len(bp.LoopBands) > 0 || bp.BossShell != nil || bp.PocketShell != nil {
		return no, false, nil
	}
	pointTerm, tiltTerm := 0.0, 0.0
	var faces []*CFace
	for _, f := range bp.Faces {
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		pp := f.View(bp.Xform)
		var face *CFace
		if f.Planar() {
			loops, err := boundarywalk.SurveyLoops(budget, boundarywalk.Profile(*f.Region))
			if err != nil {
				if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
					return no, false, err
				}
				return no, false, nil
			}
			var elems []survey2d.SurveyElem
			for _, loop := range loops {
				for _, w := range loop {
					el, ok := survey2d.WalkElem(w.SegmentWalk)
					if !ok {
						return no, false, nil
					}
					elems = append(elems, el)
				}
			}
			sign := -1.0
			if f.Outward {
				sign = 1
			}
			face = PrismPlaneCarrier(prismCarrierFrame(pp), f.Z0, sign, NewRegion2(elems))
		} else {
			w, ok := brepCarrierWalk(f.Wall)
			if !ok {
				return no, false, nil
			}
			if face, ok = PrismWallCarrier(prismCarrierFrame(pp), w); !ok {
				return no, false, nil
			}
		}
		faces = append(faces, face)
		pointTerm = math.Max(pointTerm, face.LiftRound)
		tiltTerm = math.Max(tiltTerm, BodyFaceTiltDelta([]*CFace{face}, f.Frame, bp.Xform))
	}
	delta := proofbound.AbsSumUpper(pointTerm, bp.AxialDelta(), tiltTerm)
	return BodyCarriers{Faces: faces, Delta: delta}, true, nil
}

func brepCarrierWalk(seg sectionrecord.CurveSegment) (survey2d.SegmentWalk, bool) {
	w, err := boundarywalk.WalkOf(seg, nil)
	if err != nil {
		return survey2d.SegmentWalk{}, false
	}
	_, ok := survey2d.WalkElem(w)
	return w, ok
}
