// This came from 
// https://developer.mozilla.org/en-US/docs/Web/API/Web_Speech_API/Using_the_Web_Speech_API

var synth      = window.speechSynthesis;
var voicePitch = 1.0;
var voiceRate  = 1.0;
var voices     = []; 
var voice;

function pickVoice() {
	voices = synth.getVoices();

	for (var i = 0; i < voices.length; i++) {	
		// console.log(voices[i]);	
		if (voices[i].lang == "en")  {
			console.log(voices[i]);
			voice = voices[i];
			if (voices[i].default) {
				console.log(voices[i]);
				console.log('default');
			}
		}
	}
}

function speak(txt)  {
	if (voice == undefined)  {
		console.log('I tried to say: ' + txt);
		// return;
	}

	if (synth.speaking)  {
		// alert('I am talking now');
		console.log('speechSynthesis.speaking');
		return;
	}

	var utterThis = new SpeechSynthesisUtterance(txt);
	utterThis.voice = voice;
	utterThis.pitch = voicePitch;
	utterThis.rate  = voiceRate;
	synth.speak(utterThis);
	
	utterThis.onerror = function (event)  {
		console.log('SpeechSynthesisUtterance.onerror');
    }	
    utterThis.onend = function (event) {
		console.log('SpeechSynthesisUtterance.onend');
    }    
    utterThis.onstart = function (event) {
		console.log('started');
	}	
}  

// See https://github.com/PX4/PX4-Autopilot/blob/master/src/lib/tunes/tune_definition.desc
// See https://docs.px4.io/v1.12/en/modules/modules_system.html#tune-control
/*
//           ordinal name                  tune                                        interruptable*     hint
//  * Repeated tunes should always be defined as interruptable, if not an explict 'tone_control stop' is needed
PX4_DEFINE_TUNE(0,  CUSTOM,                "",                                              true)  //  empty to align with the index 
PX4_DEFINE_TUNE(1,  STARTUP,               "MFT240L8 O4aO5dc O4aO5dc O4aO5dc L16dcdcdcdc",  true)  //  startup tune 
PX4_DEFINE_TUNE(2,  ERROR_TUNE,            "MBT200a8a8a8PaaaP",                             true)  //  ERROR tone 
PX4_DEFINE_TUNE(3,  NOTIFY_POSITIVE,       "MFT200e8a8a",                                   true)  //  Notify Positive tone 
PX4_DEFINE_TUNE(4,  NOTIFY_NEUTRAL,        "MFT200e8e",                                     true)  //  Notify Neutral tone 
PX4_DEFINE_TUNE(5,  NOTIFY_NEGATIVE,       "MFT200e8c8e8c8e8c8",                            true)  //  Notify Negative tone 
PX4_DEFINE_TUNE(6,  ARMING_WARNING,        "MNT75L1O2G",                                    false) //  arming warning 
PX4_DEFINE_TUNE(7,  BATTERY_WARNING_SLOW,  "MBNT100a8",                                     true)  //  battery warning slow 
PX4_DEFINE_TUNE(8,  BATTERY_WARNING_FAST,  "MBNT255a8a8a8a8a8a8a8a8a8a8a8a8a8a8a8a8",       true)  //  battery warning fast 
PX4_DEFINE_TUNE(9,  GPS_WARNING,           "MFT255L4AAAL1F#",                               false) //  gps warning slow 
PX4_DEFINE_TUNE(10, ARMING_FAILURE,        "MFT255L4<<<BAP",                                false) //  arming failure tune 
PX4_DEFINE_TUNE(11, PARACHUTE_RELEASE,     "MFT255L16agagagag",                             false) //  parachute release 
PX4_DEFINE_TUNE(12, SINGLE_BEEP,           "MFT100a8",                                      false) //  single beep 
PX4_DEFINE_TUNE(13, HOME_SET,              "MFT100L4>G#6A#6B#4",                            false) //  home set tune 
PX4_DEFINE_TUNE(14, SD_INIT,               "MFAGPAG",                                       false) //  Make FS 
PX4_DEFINE_TUNE(15, SD_ERROR,              "MNBG",                                          false) //  format failed 
PX4_DEFINE_TUNE(16, PROG_PX4IO,            "MLL32CP8MB",                                    false) //  Program PX4IO 
PX4_DEFINE_TUNE(17, PROG_PX4IO_OK,         "MLL8CDE",                                       false) //  Program PX4IO success 
PX4_DEFINE_TUNE(18, PROG_PX4IO_ERR,        "ML<<CP4CP4CP4CP4CP4",                           true)  //  Program PX4IO fail 
PX4_DEFINE_TUNE(19, POWER_OFF,             "MFT255a8g8f8e8c8<b8a8g4",                       true)  //  When pressing off button 
*/


// This code was copied from 
// https://firmware.ardupilot.org/Tools/ToneTester/#MFT200e8c8e8c8e8c8


// if you have another AudioContext class use that one, as some browsers have a limit
var audioCtx = new (window.AudioContext || window.webkitAudioContext || window.audioContext);

function play_note(duration, frequency, volume, type, callback) {
    var oscillator = audioCtx.createOscillator();
    var gainNode = audioCtx.createGain();

    oscillator.connect(gainNode);
    gainNode.connect(audioCtx.destination);

    if (volume){gainNode.gain.value = volume;};
    if (frequency){oscillator.frequency.value = frequency;}
    if (type){oscillator.type = type;}

    oscillator.start();
    setTimeout(function(){oscillator.stop(); callback();}, duration);
};

function play_tone(tone_str) {
    var _next = 0;
    var _tempo = 120;
    var _note_length = 4;
    var _note_mode = 'MODE_NORMAL';
    var _octave = 4;
    var _silence_length = 0;
    var _repeat = false;

    tone_str = tone_str.toUpperCase();

    function next_char() {
        while(_next < tone_str.length && /\s/.test(tone_str[_next])) {
            _next++;
        }
        if (_next >= tone_str.length) {
            return '\0';
        }
        return tone_str[_next];
    }

    function next_number() {
        number = '';
        while (!isNaN(number + next_char())) {
            number = number+next_char();
            _next++;
        }
        return parseInt(number, 10) || 0;
    }

    function next_dots() {
        var dots = 0;

        while (next_char() == '.') {
            _next++;
            dots++;
        }

        return dots;
    }

    function rest_duration(rest_length, dots) {
        var whole_note_period = (60*1000*4) / _tempo;

        if (rest_length == 0) {
            rest_length = 1;
        }

        var rest_period = whole_note_period / rest_length;
        var dot_extension = rest_period / 2;

        while (dots--) {
            rest_period += dot_extension;
            dot_extension /= 2;
        }

        return rest_period;
    }

    function next_action() {
        if (_silence_length > 0) {
            setTimeout(next_action, _silence_length);
            _silence_length = 0;
            return;
        }

        var note = 0;
        var note_length = _note_length;
        var duration = 0;

        while (note == 0) {
            var c = next_char();
            if (c == '\0') {
                return;
            }

            _next++;

            switch(c) {
                case 'L':
                    _note_length = next_number();

                    if (_note_length < 1) {
                        return;
                    }
                    break;
                case 'O':
                    _octave = next_number();

                    if (_octave > 6) {
                        _octave = 6;
                    }
                    break;
                case '<':
                    if (_octave > 0) {
                        _octave--;
                    }
                    break;
                case '>':
                    if (_octave < 6) {
                        _octave++;
                    }
                    break;
                case 'M':
                    c = next_char();
                    if (c == 0) {
                        return;
                    }

                    _next++;

                    switch (c) {
                        case 'N':
                            _note_mode = 'MODE_NORMAL';
                            break;
                        case 'L':
                            _note_mode = 'MODE_LEGATO';
                            break;
                        case 'S':
                            _note_mode = 'MODE_STACCATO';
                            break;
                        case 'F':
                            _repeat = false;
                            break;
                        case 'B':
                            _repeat = true;
                            break;
                        default:
                            return;
                    }
                    break;
                case 'P':
                    setTimeout(next_action, rest_duration(next_number(), next_dots()));
                    return;
                case 'T':
                    _tempo = next_number();
                    if (_tempo < 32 || _tempo > 255) {
                        return;
                    }
                    break;
                case 'N':
                    note = next_number();
                    if (note < 0 || note > 84) {
                        return;
                    }

                    if (note == 0) {
                        setTimeout(next_action, rest_duration(_note_length, next_dots()));
                        return;
                    }
                    break;
                case 'A':
                case 'B':
                case 'C':
                case 'D':
                case 'E':
                case 'F':
                case 'G':
                    var note_tab = {'A':9, 'B':11, 'C':0, 'D':2, 'E':4, 'F':5, 'G': 7};
                    note = note_tab[c] + (_octave*12) + 1;
                    c = next_char();
                    switch (c) {
                        case '#':
                        case '+':
                            if (note < 84) {
                                note++;
                            }
                            _next++;
                            break;
                        case '-':
                            if (note > 1) {
                                note--;
                            }
                            _next++;
                            break;
                        default:
                        break;
                    }

                    note_length = next_number();
                    if (note_length == 0) {
                        note_length = _note_length;
                    }
                    break;
                default:
                    return;
            }
        }

        var whole_note_period = (60 * 1000 * 4) / _tempo;
        if (note_length == 0) {
            note_length = 1;
        }

        var note_period = whole_note_period / note_length;

        switch(_note_mode) {
            case 'MODE_NORMAL':
                _silence_length = note_period / 8;
                break;
            case 'MODE_STACCATO':
                _silence_length = note_period / 4;
                break;
            case 'MODE_LEGATO':
            default:
                _silence_length = 0;
                break;
        }
        note_period -= _silence_length;

        var dot_extension = note_period / 2;
        var dots = next_dots();

        while (dots--) {
            note_period += dot_extension;
            dot_extension /= 2;
        }

        play_note(note_period, 880.0 * Math.exp(Math.log(2.0) * (Math.floor(note) - 46) / 12.0), 1, 'square', next_action);
    }

    next_action();
}


function speakableAssetID(assetID)  {	
	// Convert 107 to "one oh seven" or "1 oh 7"
	var string = ''
	
	assetID = parseInt(assetID);
	
	if (assetID <= 99)  {
		string = String(assetID);
	}  else if (assetID == 100)  {
		string = '1 hundred';
	}  else if (assetID <= 999)  {
		string = String(assetID)[0];
		
		if (String(assetID)[1] == '0')  {
			string += ' oh ';
			string += String(assetID)[2];
		}  else  {
			string += ' ' + String(assetID).slice(1,);
		}
	}  else if (assetID == 1000)  {
		string = '1 thousand'
	}  else if (assetID <= 9999)  {
		string = String(assetID)[0] + ' thousand '
		
		if (String(assetID)[1] != '0')  {
			string += String(assetID)[1] + ' hundred '
		} 
		
		if (String(assetID)[2] != '0')  {
			string += String(assetID).slice(2,);
		}  else  {
			string += String(assetID)[3];
		}
	}  else  {
		return String(assetID);
	} 
		
	return string
}
