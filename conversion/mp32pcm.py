#!/usr/bin/python
# -*- coding: utf-8 -*-

from pydub import AudioSegment
import wave
import numpy as np


def save_to_wav(sound ):
    sound.export('_temp_.wav', format='wav')
    return '_temp_.wav'


def load_file(file, db, rate, ch, width, start_sec, len_sec):
    print("opene file:%s" % file)
    sound = AudioSegment.from_file(file)
    sound = sound + db
    sound = sound.set_frame_rate(rate)
    sound = sound.set_channels(ch)
    sound = sound.set_sample_width(width)

    end = start_sec+len_sec
    if len_sec < 0:
        sound = sound[start_sec * 1000:]
    else:
        sound = sound[start_sec*1000:end*1000]

    return sound

def wave_to_array(wavefile, array_path):
    wave_f = wave.open(wavefile, 'rb')
    params = wave_f.getparams()
    print(params)

    # open a txt file
    fData = open(array_path, 'w')
    bytes = params.nframes * params.sampwidth * params.nchannels

    fData.write("#include <stdint.h>\n\n")
    fData.write("static const uint8_t wave_array[];\n")
    fData.write("char *wave_get(void){return (char*)wave_array;}\n")
    fData.write("uint32_t wave_get_size(void){return %d;}\n" % (bytes))
    fData.write("uint32_t wave_get_framerate(void){return %d;}\n" % params.framerate)
    fData.write("uint32_t wave_get_bits(void){return %d;}\n" % (params.sampwidth*8))
    fData.write("uint32_t wave_get_ch(void){return %d;}\n" % params.nchannels)
    fData.write("/* size : %d */\n" % (bytes))
    fData.write("static const uint8_t wave_array[]={\n")

    for i in range(int(params.nframes / 32)):
        data = wave_f.readframes(32)
        ldata = list(data)
        if params.sampwidth == 1:
            ldata = np.array(ldata)-127
            ldata = list(ldata)
        sdata = str(ldata)
        fData.writelines(sdata[1:-1])  # remove
        fData.write(",\n")
    fData.write("};\n")

    print('size=%d' % (bytes))
    fData.close()  # close data file
    wave_f.close()


if __name__ == "__main__":
    s1 = load_file("sweep44.1K.wav", db = -10, rate = 44100, ch = 1, width = 2, start_sec = 10, len_sec = 20)
    # f = save_to_wav(s1)
    # wave_to_array(f, "/home/zhouli/Work/pwm_audio/main/wave.c")
