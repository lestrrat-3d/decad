package decad

import (
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// errNilDocument is the refusal of a nil document, which a feature constructor
// records, because the operation reports it right after the nil ctx check.
var errNilDocument = fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)

// namedArg is one operand of a feature constructor for the nil check: the
// constructor parameter that names it, a noun for the message, and whether it
// is nil.
type namedArg struct {
	subject, noun string
	isNil         bool
}

// sketchArg names the sketch operand s of a feature constructor.
func sketchArg(s *sketch.Sketch) namedArg { return namedArg{"s", "sketch", s == nil} }

// profileArg names the profile operand p of a feature constructor.
func profileArg(p *sketch.Profile) namedArg { return namedArg{"p", "profile", p == nil} }

// sectionArg names a section operand, a *sketch.Profile or a *sketch.Chain,
// for the nil check. A typed nil pointer reads as nil.
func sectionArg(subject string, sec any) namedArg {
	switch v := sec.(type) {
	case *sketch.Profile:
		return namedArg{subject, "profile", v == nil}
	case *sketch.Chain:
		return namedArg{subject, "chain", v == nil}
	}
	return namedArg{subject, "section", true}
}

// recordNilFaults records the faults a sketch-fed feature reports right after
// its nil ctx check and before it reads anything else: a nil document, then
// the first nil operand among args, named as the constructor parameter
// (agent-docs/command-api-design.md rule A5). op names the operation in the
// message.
func recordNilFaults(st *opcommand.State, op string, d *Document, args ...namedArg) {
	if d == nil {
		st.Record("d", errNilDocument)
	}
	nouns := make([]string, len(args))
	for i, a := range args {
		nouns[i] = a.noun
	}
	for _, a := range args {
		if !a.isNil {
			continue
		}
		st.Record(a.subject, fmt.Errorf(`%w: %s requires a non-nil %s`, ErrDegenerate, op, joinNouns(nouns)))
		return
	}
}

// joinNouns lists nouns as "a", "a and b" or "a, b, and c".
func joinNouns(n []string) string {
	switch len(n) {
	case 0:
		return ""
	case 1:
		return n[0]
	case 2:
		return n[0] + " and " + n[1]
	}
	return strings.Join(n[:len(n)-1], ", ") + ", and " + n[len(n)-1]
}

// The shared settings of the sketch-fed feature commands. Each command declares
// its own one-line method over one of these setters
// (agent-docs/command-api-design.md §6); a method stores its value, and the
// operation's gate refuses a fault of it at the position the
// method value is validated at (§4, rules A6 and A9).

// surfaceResultSetting is the SurfaceResult setting of ExtrudeCommand,
// RevolveCommand, SweepCommand and LoftCommand. Calling the method twice is
// the same as calling it once.
type surfaceResultSetting struct{ on bool }

func (s *surfaceResultSetting) set() { s.on = true }

// extrudeSettings is what Extrude's implementation reads of its methods: the
// taper with the error validating it, and the surface flag. ExtrudeCommand,
// CutExtrudeCommand, EmbossCommand and WebPathCommand fill it, and the root
// composites that extrude a tool fill it with the taper they compute.
type extrudeSettings struct {
	taper opcommand.Slot[units.Value]
	// taperSubject names the method that stored the taper, for the diagnostic.
	// A composite that computed the taper leaves it empty.
	taperSubject string
	surface      bool
}

// withTaper returns settings that hold the taper a, as a method of a command
// would have stored it.
func withTaper(a units.Value, subject string) extrudeSettings {
	var st extrudeSettings
	st.setTaper(a, subject)
	return st
}

func (s *extrudeSettings) setTaper(a units.Value, subject string) {
	s.taper.Set(a, featureoption.ValidateTaper(a))
	s.taperSubject = subject
}

// config folds the settings. A taper fault is the Taper method's own.
func (s extrudeSettings) config() (featureoption.ExtrudeConfig, error) {
	cfg := featureoption.ExtrudeConfig{Taper: units.Degrees(0), SurfaceResult: s.surface}
	if !s.taper.IsSet() {
		return cfg, nil
	}
	taper, err := s.taper.Get()
	if err != nil {
		return featureoption.ExtrudeConfig{}, subjectError(OperationExtrude, s.taperSubject, err)
	}
	cfg.Taper = taper
	return cfg, nil
}
