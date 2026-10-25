package prooflineage

import (
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

var (
	ErrInvalidContainedCutMoments = errors.New("contained-cut moment evidence is invalid")
	ErrUnprovedContainedCutVolume = errors.New("contained-cut moment volume is not proven positive")
)

// MomentSnapshot is a density-independent V/P/Q enclosure about Anchor.
type MomentSnapshot struct {
	Anchor r3.Vec
	Volume proofbound.RatInterval
	First  [3]proofbound.RatInterval
	Second [3][3]proofbound.RatInterval
}

// SourceMoments identifies the source body that produced a snapshot.
type SourceMoments struct {
	Producer int
	Snapshot MomentSnapshot
}

// ContainedCutMoments freezes source moments and their source-derived result.
// Its rational endpoints are private and every read returns independent copies.
type ContainedCutMoments struct {
	resultProducer int
	target         SourceMoments
	tool           SourceMoments
	difference     MomentSnapshot
	containment    Containment
}

// NewContainedCutMoments validates and freezes source moments for one Cut.
func NewContainedCutMoments(resultProducer int, parent *VolumeLineage,
	target, tool SourceMoments) (*ContainedCutMoments, error) {
	if err := validateMomentParent(parent, target, tool); err != nil {
		return nil, err
	}
	targetCopy, err := cloneMomentSnapshot(target.Snapshot)
	if err != nil {
		return nil, err
	}
	toolCopy, err := cloneMomentSnapshot(tool.Snapshot)
	if err != nil {
		return nil, err
	}
	if targetCopy.Anchor != toolCopy.Anchor {
		return nil, invalidContainedCutMoments("source anchors differ")
	}
	difference, err := subtractMomentSnapshots(targetCopy, toolCopy)
	if err != nil {
		return nil, err
	}
	if difference.Volume.Hi.Sign() <= 0 {
		return nil, invalidContainedCutMoments("source volume difference is nonpositive")
	}
	if difference.Volume.Lo.Sign() <= 0 {
		return nil, ErrUnprovedContainedCutVolume
	}
	if err := validateMomentVolumeLinks(parent, targetCopy, toolCopy, difference); err != nil {
		return nil, err
	}
	return &ContainedCutMoments{
		resultProducer: resultProducer,
		target:         SourceMoments{Producer: target.Producer, Snapshot: targetCopy},
		tool:           SourceMoments{Producer: tool.Producer, Snapshot: toolCopy},
		difference:     difference,
		containment:    parent.Containment,
	}, nil
}

// Read validates the frozen identities against their owning lineage and
// returns independent rational copies of the source and difference snapshots.
func (m *ContainedCutMoments) Read(resultProducer int, parent *VolumeLineage) (
	target, tool, difference MomentSnapshot, err error) {
	if m == nil {
		return MomentSnapshot{}, MomentSnapshot{}, MomentSnapshot{}, invalidContainedCutMoments("missing moment record")
	}
	if resultProducer != m.resultProducer {
		return MomentSnapshot{}, MomentSnapshot{}, MomentSnapshot{}, invalidContainedCutMoments("result producer changed")
	}
	if err := validateMomentParent(parent, m.target, m.tool); err != nil {
		return MomentSnapshot{}, MomentSnapshot{}, MomentSnapshot{}, err
	}
	if parent.Containment != m.containment {
		return MomentSnapshot{}, MomentSnapshot{}, MomentSnapshot{}, invalidContainedCutMoments("containment facts changed")
	}
	target, err = cloneMomentSnapshot(m.target.Snapshot)
	if err != nil {
		return MomentSnapshot{}, MomentSnapshot{}, MomentSnapshot{}, err
	}
	tool, err = cloneMomentSnapshot(m.tool.Snapshot)
	if err != nil {
		return MomentSnapshot{}, MomentSnapshot{}, MomentSnapshot{}, err
	}
	difference, err = cloneMomentSnapshot(m.difference)
	if err != nil {
		return MomentSnapshot{}, MomentSnapshot{}, MomentSnapshot{}, err
	}
	recomputed, err := subtractMomentSnapshots(target, tool)
	if err != nil || !equalMomentSnapshots(recomputed, difference) {
		return MomentSnapshot{}, MomentSnapshot{}, MomentSnapshot{}, invalidContainedCutMoments("stored difference does not match sources")
	}
	if err := validateMomentVolumeLinks(parent, target, tool, difference); err != nil {
		return MomentSnapshot{}, MomentSnapshot{}, MomentSnapshot{}, err
	}
	return target, tool, difference, nil
}

func validateMomentParent(parent *VolumeLineage, target, tool SourceMoments) error {
	if parent == nil {
		return invalidContainedCutMoments("missing parent volume lineage")
	}
	if target.Producer != parent.TargetProducer || tool.Producer != parent.ToolProducer {
		return invalidContainedCutMoments("source producer does not match volume lineage")
	}
	if err := validateContainment(parent.Containment); err != nil {
		return invalidContainedCutMoments("parent containment facts are invalid")
	}
	if target.Snapshot.Anchor != tool.Snapshot.Anchor {
		return invalidContainedCutMoments("source anchors differ")
	}
	if err := validateMomentSnapshot(target.Snapshot); err != nil {
		return err
	}
	if err := validateMomentSnapshot(tool.Snapshot); err != nil {
		return err
	}
	return nil
}

func validateMomentVolumeLinks(parent *VolumeLineage, target, tool, difference MomentSnapshot) error {
	targetReading, ok := measurementInterval(parent.Target)
	if !ok || !intervalsOverlap(target.Volume, targetReading) {
		return invalidContainedCutMoments("target source volume disagrees with parent reading")
	}
	toolReading, ok := measurementInterval(parent.Tool)
	if !ok || !intervalsOverlap(tool.Volume, toolReading) {
		return invalidContainedCutMoments("tool source volume disagrees with parent reading")
	}
	for _, reading := range []measurement.Measurement{parent.Ordinary, parent.Derived} {
		iv, valid := measurementInterval(reading)
		if !valid || !intervalsOverlap(difference.Volume, iv) {
			return invalidContainedCutMoments("source difference volume disagrees with parent reading")
		}
	}
	return nil
}

func measurementInterval(reading measurement.Measurement) (proofbound.RatInterval, bool) {
	if reading.Value.Unit() != units.CubicMillimeter || reading.Bound.Unit() != units.CubicMillimeter ||
		!measurement.FiniteValues(reading.Value.Base(), reading.Bound.Base()) || reading.Bound.Base() < 0 {
		return proofbound.RatInterval{}, false
	}
	value, bound := proofarith.FloatRat(reading.Value.Base()), proofarith.FloatRat(reading.Bound.Base())
	if value == nil || bound == nil {
		return proofbound.RatInterval{}, false
	}
	return proofbound.Interval(new(big.Rat).Sub(value, bound), new(big.Rat).Add(value, bound)), true
}

func validateMomentSnapshot(snapshot MomentSnapshot) error {
	if !proofbound.FiniteVec(snapshot.Anchor) {
		return invalidContainedCutMoments("anchor is not finite")
	}
	for _, interval := range momentIntervals(snapshot) {
		if interval.Lo == nil || interval.Hi == nil || interval.Lo.Cmp(interval.Hi) > 0 {
			return invalidContainedCutMoments("moment interval has invalid endpoints")
		}
	}
	for i := range 3 {
		for j := range 3 {
			if snapshot.Second[i][j].Lo.Cmp(snapshot.Second[j][i].Lo) != 0 ||
				snapshot.Second[i][j].Hi.Cmp(snapshot.Second[j][i].Hi) != 0 {
				return invalidContainedCutMoments("second-moment matrix is not symmetric")
			}
		}
	}
	return nil
}

func momentIntervals(snapshot MomentSnapshot) []proofbound.RatInterval {
	intervals := make([]proofbound.RatInterval, 0, 13)
	intervals = append(intervals, snapshot.Volume)
	intervals = append(intervals, snapshot.First[:]...)
	for i := range 3 {
		intervals = append(intervals, snapshot.Second[i][:]...)
	}
	return intervals
}

func cloneMomentSnapshot(snapshot MomentSnapshot) (MomentSnapshot, error) {
	if err := validateMomentSnapshot(snapshot); err != nil {
		return MomentSnapshot{}, err
	}
	out := MomentSnapshot{Anchor: snapshot.Anchor}
	var ok bool
	if out.Volume, ok = cloneMomentInterval(snapshot.Volume); !ok {
		return MomentSnapshot{}, invalidContainedCutMoments("volume endpoints are missing")
	}
	for i := range out.First {
		if out.First[i], ok = cloneMomentInterval(snapshot.First[i]); !ok {
			return MomentSnapshot{}, invalidContainedCutMoments("first-moment endpoints are missing")
		}
		for j := range out.Second[i] {
			if out.Second[i][j], ok = cloneMomentInterval(snapshot.Second[i][j]); !ok {
				return MomentSnapshot{}, invalidContainedCutMoments("second-moment endpoints are missing")
			}
		}
	}
	return out, nil
}

func cloneMomentInterval(interval proofbound.RatInterval) (proofbound.RatInterval, bool) {
	if interval.Lo == nil || interval.Hi == nil {
		return proofbound.RatInterval{}, false
	}
	return proofbound.Interval(interval.Lo, interval.Hi), true
}

func subtractMomentSnapshots(target, tool MomentSnapshot) (MomentSnapshot, error) {
	if target.Anchor != tool.Anchor {
		return MomentSnapshot{}, invalidContainedCutMoments("source anchors differ")
	}
	difference := MomentSnapshot{Anchor: target.Anchor,
		Volume: proofbound.IntervalSub(target.Volume, tool.Volume)}
	for i := range 3 {
		difference.First[i] = proofbound.IntervalSub(target.First[i], tool.First[i])
		for j := i; j < 3; j++ {
			entry := proofbound.IntervalSub(target.Second[i][j], tool.Second[i][j])
			difference.Second[i][j], difference.Second[j][i] = entry, entry
		}
	}
	return difference, validateMomentSnapshot(difference)
}

func equalMomentSnapshots(a, b MomentSnapshot) bool {
	if a.Anchor != b.Anchor || !equalMomentInterval(a.Volume, b.Volume) {
		return false
	}
	for i := range 3 {
		if !equalMomentInterval(a.First[i], b.First[i]) {
			return false
		}
		for j := range 3 {
			if !equalMomentInterval(a.Second[i][j], b.Second[i][j]) {
				return false
			}
		}
	}
	return true
}

func equalMomentInterval(a, b proofbound.RatInterval) bool {
	return a.Lo != nil && a.Hi != nil && b.Lo != nil && b.Hi != nil &&
		a.Lo.Cmp(b.Lo) == 0 && a.Hi.Cmp(b.Hi) == 0
}

func intervalsOverlap(a, b proofbound.RatInterval) bool {
	return a.Lo.Cmp(b.Hi) <= 0 && b.Lo.Cmp(a.Hi) <= 0
}

func invalidContainedCutMoments(reason string) error {
	return fmt.Errorf("%w: %s", ErrInvalidContainedCutMoments, reason)
}
