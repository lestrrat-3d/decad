package facetedrecord

import (
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/prooflineage"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
)

var ErrInvalidExactCopyMoments = errors.New("exact-copy moment evidence is invalid")

// ExactCopyMomentRecord freezes contained-Cut source moments for one direct
// public exact-rotation Copy. Placement, scaling, booleans, and modifications
// drop this record; only the authorized Copy capture may create a new one.
type ExactCopyMomentRecord struct {
	sourceProducer  provenance.Producer
	resultProducer  provenance.Producer
	sourcePlacement r3.Transform
	resultPlacement r3.Transform
	parent          *prooflineage.VolumeLineage
	rotation        *exactrotation.Rotation
}

// NewExactCopyMomentRecord validates and freezes contained-Cut evidence for
// one direct exact-rotation Copy.
func NewExactCopyMomentRecord(sourceProducer, resultProducer provenance.Producer,
	sourcePlacement, resultPlacement r3.Transform, parent *prooflineage.VolumeLineage,
	rotation *exactrotation.Rotation) (*ExactCopyMomentRecord, error) {
	if parent == nil || parent.Moments == nil {
		return nil, invalidExactCopyMoments("source volume lineage has no moments")
	}
	if sourceProducer <= 0 || resultProducer <= 0 || sourceProducer == resultProducer {
		return nil, invalidExactCopyMoments("source and result producers are invalid")
	}
	if rotation == nil {
		return nil, invalidExactCopyMoments("exact rotation is missing")
	}
	if sourcePlacement != r3.Identity() || !resultPlacement.IsValid() {
		return nil, invalidExactCopyMoments("source or result placement is invalid")
	}
	linear, translation := rotation.Linear(), rotation.Translation()
	if err := validateExactRotation(linear, translation); err != nil {
		return nil, err
	}
	if _, _, _, err := parent.Moments.Read(int(sourceProducer), parent); err != nil {
		return nil, invalidExactCopyMoments("source moment lineage is invalid: %v", err)
	}
	privateParent := &prooflineage.VolumeLineage{
		TargetProducer: parent.TargetProducer,
		ToolProducer:   parent.ToolProducer,
		Target:         parent.Target,
		Tool:           parent.Tool,
		Ordinary:       parent.Ordinary,
		Derived:        parent.Derived,
		Containment:    parent.Containment,
	}
	target, tool, _, err := parent.Moments.Read(int(sourceProducer), parent)
	if err != nil {
		return nil, invalidExactCopyMoments("source moment lineage changed: %v", err)
	}
	privateParent.Moments, err = prooflineage.NewContainedCutMoments(int(sourceProducer), privateParent,
		prooflineage.SourceMoments{Producer: privateParent.TargetProducer, Snapshot: target},
		prooflineage.SourceMoments{Producer: privateParent.ToolProducer, Snapshot: tool})
	if err != nil {
		return nil, invalidExactCopyMoments("source moments could not be frozen: %v", err)
	}
	return &ExactCopyMomentRecord{
		sourceProducer: sourceProducer, resultProducer: resultProducer,
		sourcePlacement: sourcePlacement, resultPlacement: resultPlacement,
		parent: privateParent, rotation: rotation,
	}, nil
}

// Read validates the receiving Copy and returns fresh source-difference
// moments and exact rotation coefficients.
func (r *ExactCopyMomentRecord) Read(resultProducer provenance.Producer,
	resultPlacement r3.Transform) (prooflineage.MomentSnapshot, [3][3]*big.Rat, [3]*big.Rat, error) {
	if r == nil {
		return prooflineage.MomentSnapshot{}, [3][3]*big.Rat{}, [3]*big.Rat{},
			invalidExactCopyMoments("record is missing")
	}
	if resultProducer != r.resultProducer || resultPlacement != r.resultPlacement ||
		r.sourcePlacement != r3.Identity() || !resultPlacement.IsValid() {
		return prooflineage.MomentSnapshot{}, [3][3]*big.Rat{}, [3]*big.Rat{},
			invalidExactCopyMoments("receiving producer or placement changed")
	}
	if r.parent == nil || r.parent.Moments == nil || r.rotation == nil {
		return prooflineage.MomentSnapshot{}, [3][3]*big.Rat{}, [3]*big.Rat{},
			invalidExactCopyMoments("frozen source record is incomplete")
	}
	target, tool, difference, err := r.parent.Moments.Read(int(r.sourceProducer), r.parent)
	if err != nil {
		return prooflineage.MomentSnapshot{}, [3][3]*big.Rat{}, [3]*big.Rat{},
			invalidExactCopyMoments("frozen source moments are invalid: %v", err)
	}
	frozen, err := prooflineage.NewContainedCutMoments(int(r.sourceProducer), r.parent,
		prooflineage.SourceMoments{Producer: r.parent.TargetProducer, Snapshot: target},
		prooflineage.SourceMoments{Producer: r.parent.ToolProducer, Snapshot: tool})
	if err != nil {
		return prooflineage.MomentSnapshot{}, [3][3]*big.Rat{}, [3]*big.Rat{},
			invalidExactCopyMoments("frozen source moments cannot be reconstructed: %v", err)
	}
	_, _, reread, err := frozen.Read(int(r.sourceProducer), r.parent)
	if err != nil || !sameMomentSnapshot(difference, reread) {
		return prooflineage.MomentSnapshot{}, [3][3]*big.Rat{}, [3]*big.Rat{},
			invalidExactCopyMoments("frozen difference no longer matches its sources")
	}
	linear, translation := r.rotation.Linear(), r.rotation.Translation()
	if err := validateExactRotation(linear, translation); err != nil {
		return prooflineage.MomentSnapshot{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
	}
	return difference, linear, translation, nil
}

func validateExactRotation(linear [3][3]*big.Rat, translation [3]*big.Rat) error {
	for i := range 3 {
		if translation[i] == nil {
			return invalidExactCopyMoments("rotation translation coefficient %d is missing", i)
		}
		for j := range 3 {
			if linear[i][j] == nil {
				return invalidExactCopyMoments("rotation coefficient %d,%d is missing", i, j)
			}
		}
	}
	if massmoment.OrthonormalityDefect(linear).Sign() != 0 ||
		massmoment.Determinant(linear).Cmp(big.NewRat(1, 1)) != 0 {
		return invalidExactCopyMoments("rotation is not exactly orthogonal with determinant +1")
	}
	return nil
}

func sameMomentSnapshot(a, b prooflineage.MomentSnapshot) bool {
	if a.Anchor != b.Anchor || !sameMomentInterval(a.Volume, b.Volume) {
		return false
	}
	for i := range 3 {
		if !sameMomentInterval(a.First[i], b.First[i]) {
			return false
		}
		for j := range 3 {
			if !sameMomentInterval(a.Second[i][j], b.Second[i][j]) {
				return false
			}
		}
	}
	return true
}

func sameMomentInterval(a, b proofbound.RatInterval) bool {
	return a.Lo != nil && a.Hi != nil && b.Lo != nil && b.Hi != nil &&
		a.Lo.Cmp(b.Lo) == 0 && a.Hi.Cmp(b.Hi) == 0
}

func invalidExactCopyMoments(format string, args ...any) error {
	return fmt.Errorf("%w: %s", ErrInvalidExactCopyMoments, fmt.Sprintf(format, args...))
}
