package parts

import (
	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Cause names why a joint is not Confirmed.
type Cause int

const (
	CauseNone           Cause = iota
	CauseNoMatch              // no face pair of the joint's form exists, and every face near the pair was read
	CauseClearance            // the measured clearance is not the declared one
	CauseAxis                 // the matched axis is not the declared one
	CauseInterference         // a pin larger than its bore with no declared press fit
	CauseApart                // the parts are proven farther apart than the joint allows
	CauseTooClose             // a keepout pair is proven closer than its minimum gap
	CauseUnreadFace           // a face near the pair reads Faceted or the body would not mesh
	CauseNoWitness            // no point proves the contact or the face overlap
	CauseNeedsPairProof       // only a pair proof could decide it
)

// String returns the lower-case words for the cause.
func (c Cause) String() string {
	switch c {
	case CauseNoMatch:
		return "no match"
	case CauseClearance:
		return "clearance"
	case CauseAxis:
		return "axis"
	case CauseInterference:
		return "interference"
	case CauseApart:
		return "apart"
	case CauseTooClose:
		return "too close"
	case CauseUnreadFace:
		return "unread face"
	case CauseNoWitness:
		return "no witness"
	case CauseNeedsPairProof:
		return "needs pair proof"
	}
	return "none"
}

// AxisSource says where a joint's axis came from.
type AxisSource int

const (
	AxisSourceNone     AxisSource = iota // no axis
	AxisSourceFaces                      // the matched cylinder faces
	AxisSourceDeclared                   // the joint's declared Axis
)

// String returns the lower-case words for the source.
func (a AxisSource) String() string {
	switch a {
	case AxisSourceFaces:
		return "faces"
	case AxisSourceDeclared:
		return "declared"
	}
	return "none"
}

// JointReport is what Check found.
type JointReport struct {
	Request     CheckRequest  // the effective tolerances and options
	Joints      []JointResult // one per declared joint, in Join order
	Unexplained []PairFinding // part pairs whose boxes meet and that no joint names
}

// CheckRequest is the effective settings of one Check.
type CheckRequest struct {
	FitTolerance, ContactTolerance, AxisTolerance, MaxFit units.Value
	WitnessSearch                                         bool
	VerifyReport                                          bool       // a WithVerifyReport report was read
	Meshes                                                []PartMesh // one per part with a body, in Set order
}

// PartMesh says how one part's faces were read.
type PartMesh struct {
	Part      string
	Tolerance units.Value // the chord tolerance the part was read at
	Bound     units.Value // that mesh's bound
	Verified  bool        // the mesh was built at VerifyAll
}

// JointResult is what Check found for one declared joint.
type JointResult struct {
	Joint    Joint
	Status   JointStatus
	Cause    Cause              // why it is not Confirmed; CauseNone when it is
	Axis     *Axis              // PinInBore, PinInSlot: the axis used
	AxisFrom AxisSource         // faces, declared or none
	Measured *decad.Measurement // PinInBore: R − r; PinInSlot: width − 2r; FaceContact: the planes' distance
	Engaged  *decad.Measurement // PinInBore, PinInSlot: axial overlap of the matched faces
	Witness  *r3.Vec            // a point the face overlap claim was read at
	FacesA   []*decad.Face      // the faces of A the rule read
	FacesB   []*decad.Face
	Reason   string // the numbers behind Cause, for people, not for branching
}

// PairFinding is a pair of parts whose boxes meet and that no joint names.
type PairFinding struct {
	A, B    string
	Overlap decad.Box // where the two boxes, grown by the contact tolerance and their bounds, meet
}
