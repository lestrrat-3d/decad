// Package stepread reads the entity graph of an ISO 10303-21 (STEP physical
// file) into plain values (agent-docs/step-import-design.md). It knows the
// syntax and nothing of any application protocol.
package stepread

import (
	"errors"
	"fmt"
	"strconv"
	"strings"
)

// ErrMalformed marks input that is not a well-formed STEP file.
var ErrMalformed = errors.New("stepread: malformed STEP file")

// Value is one parameter: a Ref, String, Number, Enum, List, Typed, Null or
// Omitted.
type Value interface{ value() }

// Ref is a reference to the entity #N.
type Ref int

// String is a quoted string, its ” escape resolved.
type String string

// Number is an integer or a real.
type Number float64

// Enum is an enumeration or logical value without its dots: .T. is Enum("T").
type Enum string

// List is a parenthesised list.
type List []Value

// Typed is a value wrapped in a type name, such as LENGTH_MEASURE(1.5).
type Typed struct {
	Name   string
	Values List
}

// Null is the undefined value $.
type Null struct{}

// Omitted is the derived value *.
type Omitted struct{}

func (Ref) value()     {}
func (String) value()  {}
func (Number) value()  {}
func (Enum) value()    {}
func (List) value()    {}
func (Typed) value()   {}
func (Null) value()    {}
func (Omitted) value() {}

// Entity is one #N = NAME(params) instance. A complex instance, written as a
// parenthesised run of several names, holds each in Parts and has no Name.
type Entity struct {
	ID     int
	Name   string
	Params List
	Parts  []Part
}

// Part is one named member of a complex entity instance.
type Part struct {
	Name   string
	Params List
}

// Model is the DATA section's instances by number.
type Model struct {
	Entities map[int]Entity
}

// Parse reads a STEP file. It reads the first DATA section and ignores the
// header; comments are skipped.
func Parse(data []byte) (*Model, error) {
	p := &parser{src: string(data)}
	start := strings.Index(p.src, "DATA;")
	if start < 0 {
		return nil, fmt.Errorf("%w: no DATA section", ErrMalformed)
	}
	p.pos = start + len("DATA;")
	model := &Model{Entities: map[int]Entity{}}
	for {
		p.skip()
		if p.eof() {
			return nil, fmt.Errorf("%w: the DATA section is not closed by ENDSEC", ErrMalformed)
		}
		if strings.HasPrefix(p.src[p.pos:], "ENDSEC") {
			return model, nil
		}
		e, err := p.entity()
		if err != nil {
			return nil, err
		}
		if _, dup := model.Entities[e.ID]; dup {
			return nil, fmt.Errorf("%w: entity #%d is defined twice", ErrMalformed, e.ID)
		}
		model.Entities[e.ID] = e
	}
}

type parser struct {
	src string
	pos int
}

func (p *parser) eof() bool { return p.pos >= len(p.src) }

// skip passes white space and /* comments */.
func (p *parser) skip() {
	for !p.eof() {
		switch c := p.src[p.pos]; {
		case c == ' ' || c == '\t' || c == '\n' || c == '\r':
			p.pos++
		case c == '/' && strings.HasPrefix(p.src[p.pos:], "/*"):
			end := strings.Index(p.src[p.pos+2:], "*/")
			if end < 0 {
				p.pos = len(p.src)
				return
			}
			p.pos += end + 4
		default:
			return
		}
	}
}

func (p *parser) fail(format string, args ...any) error {
	return fmt.Errorf("%w: at byte %d: %s", ErrMalformed, p.pos, fmt.Sprintf(format, args...))
}

func (p *parser) expect(c byte) error {
	p.skip()
	if p.eof() || p.src[p.pos] != c {
		return p.fail("expected %q", c)
	}
	p.pos++
	return nil
}

func (p *parser) entity() (Entity, error) {
	p.skip()
	if p.eof() || p.src[p.pos] != '#' {
		return Entity{}, p.fail("expected an entity number")
	}
	p.pos++
	id, err := p.integer()
	if err != nil {
		return Entity{}, err
	}
	if err := p.expect('='); err != nil {
		return Entity{}, err
	}
	p.skip()
	e := Entity{ID: id}
	if !p.eof() && p.src[p.pos] == '(' {
		// A complex instance: a run of NAME(params) groups.
		p.pos++
		for {
			p.skip()
			if !p.eof() && p.src[p.pos] == ')' {
				p.pos++
				break
			}
			name := p.keyword()
			if name == "" {
				return Entity{}, p.fail("expected an entity name")
			}
			params, err := p.paramList()
			if err != nil {
				return Entity{}, err
			}
			e.Parts = append(e.Parts, Part{Name: name, Params: params})
		}
	} else {
		e.Name = p.keyword()
		if e.Name == "" {
			return Entity{}, p.fail("expected an entity name")
		}
		if e.Params, err = p.paramList(); err != nil {
			return Entity{}, err
		}
	}
	if err := p.expect(';'); err != nil {
		return Entity{}, err
	}
	return e, nil
}

func (p *parser) keyword() string {
	p.skip()
	start := p.pos
	for !p.eof() {
		c := p.src[p.pos]
		if c >= 'A' && c <= 'Z' || c >= 'a' && c <= 'z' || c >= '0' && c <= '9' || c == '_' || c == '-' {
			p.pos++
			continue
		}
		break
	}
	return strings.ToUpper(p.src[start:p.pos])
}

func (p *parser) integer() (int, error) {
	start := p.pos
	for !p.eof() && p.src[p.pos] >= '0' && p.src[p.pos] <= '9' {
		p.pos++
	}
	n, err := strconv.Atoi(p.src[start:p.pos])
	if err != nil {
		return 0, p.fail("expected an integer")
	}
	return n, nil
}

// paramList reads "(" value {"," value} ")".
func (p *parser) paramList() (List, error) {
	if err := p.expect('('); err != nil {
		return nil, err
	}
	var out List
	p.skip()
	if !p.eof() && p.src[p.pos] == ')' {
		p.pos++
		return out, nil
	}
	for {
		v, err := p.parseValue()
		if err != nil {
			return nil, err
		}
		out = append(out, v)
		p.skip()
		if p.eof() {
			return nil, p.fail("unterminated list")
		}
		switch p.src[p.pos] {
		case ',':
			p.pos++
		case ')':
			p.pos++
			return out, nil
		default:
			return nil, p.fail("expected ',' or ')'")
		}
	}
}

func (p *parser) parseValue() (Value, error) {
	p.skip()
	if p.eof() {
		return nil, p.fail("a value was expected")
	}
	switch c := p.src[p.pos]; {
	case c == '#':
		p.pos++
		n, err := p.integer()
		return Ref(n), err
	case c == '\'':
		return p.str()
	case c == '$':
		p.pos++
		return Null{}, nil
	case c == '*':
		p.pos++
		return Omitted{}, nil
	case c == '(':
		return p.paramList()
	case c == '.':
		p.pos++
		start := p.pos
		for !p.eof() && p.src[p.pos] != '.' {
			p.pos++
		}
		if p.eof() {
			return nil, p.fail("unterminated enumeration")
		}
		name := p.src[start:p.pos]
		p.pos++
		return Enum(strings.ToUpper(name)), nil
	case c == '-' || c == '+' || c >= '0' && c <= '9':
		return p.number()
	case c >= 'A' && c <= 'Z' || c >= 'a' && c <= 'z':
		name := p.keyword()
		params, err := p.paramList()
		return Typed{Name: name, Values: params}, err
	default:
		return nil, p.fail("unexpected %q", c)
	}
}

func (p *parser) str() (Value, error) {
	p.pos++ // the opening quote
	var sb strings.Builder
	for !p.eof() {
		c := p.src[p.pos]
		if c == '\'' {
			if p.pos+1 < len(p.src) && p.src[p.pos+1] == '\'' {
				sb.WriteByte('\'')
				p.pos += 2
				continue
			}
			p.pos++
			return String(sb.String()), nil
		}
		sb.WriteByte(c)
		p.pos++
	}
	return nil, p.fail("unterminated string")
}

func (p *parser) number() (Value, error) {
	start := p.pos
	if p.src[p.pos] == '-' || p.src[p.pos] == '+' {
		p.pos++
	}
	for !p.eof() {
		c := p.src[p.pos]
		if c >= '0' && c <= '9' || c == '.' || c == 'E' || c == 'e' {
			p.pos++
			continue
		}
		if (c == '-' || c == '+') && p.pos > start && (p.src[p.pos-1] == 'E' || p.src[p.pos-1] == 'e') {
			p.pos++
			continue
		}
		break
	}
	f, err := strconv.ParseFloat(p.src[start:p.pos], 64)
	if err != nil {
		return nil, p.fail("%q is not a number", p.src[start:p.pos])
	}
	return Number(f), nil
}
