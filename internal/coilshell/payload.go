package coilshell

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// StationsPerTurn is agent-docs/helix-design.md §5.2's station density: 256
// stations per turn, a power of two, keeps the chord departure
// ρ·(π/256)²/2 ≈ 7.5e-5·ρ two decades below the default Verify tolerance.
const StationsPerTurn = 256

// ArcChordsPerTurn is the number of chords a full turn of a circular profile
// segment is cut into (agent-docs/helix-design.md §5.4, §11 PR 3): an arc of sweep
// Δ takes ⌈ArcChordsPerTurn·Δ/2π⌉. Every chord cell enters the crossing audit
// beside the helix stations, so the count is held where a round-wire spring of
// a few turns stays inside the audit's pair budget.
const ArcChordsPerTurn = 16

// MaxStations is Table CS row CS8's station ceiling: 128 turns at
// StationsPerTurn.
const MaxStations = 1 << 15

// Payload is the evaluator's record of a coil (agent-docs/helix-design.md): the
// authenticated profile and its plane frame, the axis resolved into the plane
// with the side that puts the profile on its positive side, the pitch, the
// turn count, the hand and the accumulated placement — what a placement
// re-runs §5 from — plus the build that record produced. Build evaluates it
// over topology nodes and it implements topo.Payload
// (agent-docs/evaluator-design.md §8).
//
// Verts is the held vertex table, station-major (§5.3); VertexBound is each
// vertex's β of §5.4 and Delta the largest of them; MaxRound is the largest
// station rounding; Tris is the globally oriented held triangle set, walls
// first, then capStart and capEnd.
type Payload struct {
	Profile  momentinput.Profile
	Frame    r3.Frame
	Line     revolveaxis.Line2
	Side     int
	Pitch    float64
	Turns    float64
	LeftHand bool
	Xform    r3.Transform

	Verts       []r3.Vec
	VertexBound []float64
	Tris        [][3]int
	Delta       float64
	MaxRound    float64
}

// Transform is the accumulated rigid placement.
func (cp Payload) Transform() r3.Transform { return cp.Xform }

// ProvesSimple implements topo.SimpleProver. CP5 proves the true coil simple
// from CS5 and CS6, and the build's crossing audit proves its held shell
// embedded (agent-docs/helix-design.md Table CD row CD1). A coil is always a solid,
// so the sheet audit never reaches this method; it states the answer for any
// caller that asks.
func (cp Payload) ProvesSimple(context.Context) bool { return true }

// Record supplies the resolved axis and recorded sweep to the shell builder.
func (cp Payload) Record() (Record, error) {
	return Read(Input{
		Profile: cp.Profile, Frame: cp.Frame, Transform: cp.Xform,
		Axis: AxisInput{
			AU: cp.Line.AU, AV: cp.Line.AV, AUBound: cp.Line.AUBound, AVBound: cp.Line.AVBound,
			DU: cp.Line.DU, DV: cp.Line.DV, DUBound: cp.Line.DUBound, DVBound: cp.Line.DVBound,
			Side: cp.Side,
		},
		Pitch: cp.Pitch, Turns: cp.Turns, LeftHand: cp.LeftHand,
		StationsPerTurn: StationsPerTurn, MaxStations: MaxStations,
		ArcChordsPerTurn: ArcChordsPerTurn,
	})
}

// Placed re-runs §5 from the record under the composed motion (§5.6, Table CD
// row CD7): the stations, the held table, β, δ, the orientation and the audit
// are rebuilt, so δ never accumulates across placements. Volume and Area
// read the record's closed forms under the composed map (§5.3), so they move
// only by that map's determinant and defect.
func (cp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	return BuildBody(ctx, env, Payload{
		Profile:  cp.Profile,
		Frame:    cp.Frame,
		Line:     cp.Line,
		Side:     cp.Side,
		Pitch:    cp.Pitch,
		Turns:    cp.Turns,
		LeftHand: cp.LeftHand,
		Xform:    composed,
	})
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/verifypublish): the root type name this payload had before it moved.
func (Payload) SurveyClass() string { return "coilPayload" }
