package reportvocab

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// JointBox gives each listed link one range of joint values.
type JointBox[LinkT any] []JointRange[LinkT]

// JointRange is one link's joint range in a JointBox.
type JointRange[LinkT any] struct {
	Link     LinkT
	Min, Max units.Value
}

// JointConfiguration records every link's joint value, proven half-width,
// and world pose in linkage order.
type JointConfiguration struct {
	Values []units.Value
	Bounds []units.Value
	Poses  []r3.Transform
}

// JointBoxReport records VerifyJointBox's subdivision, pair results, and verdict.
type JointBoxReport[BodyT comparable, LinkageT, LinkT, BoxT, CellT any] struct {
	Request           JointBoxRequest
	ReadingResolution units.Value
	Linkage           LinkageT
	Box               BoxT
	Links             []LinkT
	Against           []BodyT
	JointContacts     []DiagnosticPair[BodyT]
	Cells             []JointCellResult[BodyT, CellT]
	CellsEvaluated    int
	Collisions        []JointBoxCollision[BodyT]
	Clearance         *ScalarReading
	Assessment        Assessment
	Diagnostics       []Diagnostic[BodyT, CellT]
	Status            Status
}

// Passed reports whether the report is Sound. It returns false for nil.
func (r *JointBoxReport[BodyT, LinkageT, LinkT, BoxT, CellT]) Passed() bool {
	return r != nil && r.Status == Sound
}

// JointBoxRequest records the effective settings of a VerifyJointBox call.
type JointBoxRequest struct {
	RelativeTolerance units.Value
	Resolution        units.Value
	MinClearance      *units.Value
	CellBudget        int
}

// JointCell is one closed box of joint values, in linkage order.
type JointCell struct {
	Min, Max []units.Value
}

// JointCellResult records one leaf cell and its centre's pair findings.
// Clearance is a proven lower bound only when Outcome is CellClear.
type JointCellResult[BodyT comparable, CellT any] struct {
	Cell          JointCell
	Outcome       CellOutcome
	Center        JointConfiguration
	Interferences []Interference[BodyT]
	Clearances    []Clearance[BodyT]
	Diagnostics   []Diagnostic[BodyT, CellT]
	Clearance     *measurement.Measurement
}

// CellOutcome states what a JointCellResult proves.
type CellOutcome int

const (
	CellNotEvaluated CellOutcome = iota
	CellClear
	CellBlocked
	CellColliding
	CellUndecided
)

// String renders the stable lower-snake token, including unknown values.
func (o CellOutcome) String() string {
	switch o {
	case CellNotEvaluated:
		return tokenNotEvaluated
	case CellClear:
		return "clear"
	case CellBlocked:
		return "blocked"
	case CellColliding:
		return "colliding"
	case CellUndecided:
		return tokenUndecided
	default:
		return fmt.Sprintf("cell_outcome(%d)", int(o))
	}
}

// JointBoxCollision is a proven overlap at one evaluated centre.
type JointBoxCollision[BodyT comparable] struct {
	Configuration JointConfiguration
	A, B          BodyT
	Volume        measurement.Measurement
}
