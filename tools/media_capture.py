"""Local captioned MP4 writer; no download, network, or live device access."""
from pathlib import Path
import json
import shutil
import subprocess

from PIL import Image, ImageDraw, ImageFont

WIDTH, HEIGHT, VIEW_HEIGHT = 1280, 720, 640
FONT = 'C:/Windows/Fonts/malgun.ttf'


def captioned(rgb, title, subtitle, footer):
    view=Image.fromarray(rgb[:,:,:3].astype('uint8'))
    if view.size!=(WIDTH,VIEW_HEIGHT):
        view.thumbnail((WIDTH,VIEW_HEIGHT),Image.Resampling.LANCZOS)
    canvas=Image.new('RGB',(WIDTH,HEIGHT),'#f3f5f8')
    canvas.paste(view,((WIDTH-view.width)//2,48+(VIEW_HEIGHT-view.height)//2))
    draw=ImageDraw.Draw(canvas)
    draw.rectangle((0,0,WIDTH,48),fill='#15283b')
    draw.text((20,9),title,font=ImageFont.truetype(FONT,23),fill='white')
    draw.text((WIDTH-20,13),subtitle,font=ImageFont.truetype(FONT,17),fill='#d4e5ef',anchor='ra')
    draw.text((20,693),footer,font=ImageFont.truetype(FONT,15),fill='#243c51')
    return canvas


class VideoWriter:
    def __init__(self,path,fps):
        self.path=Path(path); self.fps=fps; self.count=0
        self.path.parent.mkdir(parents=True,exist_ok=True)
        executable=shutil.which('ffmpeg')
        if not executable: raise RuntimeError('Existing ffmpeg executable not found')
        self.log=self.path.with_suffix('.encode.log').open('wb')
        self.proc=subprocess.Popen([executable,'-hide_banner','-loglevel','error','-nostdin','-n',
            '-f','rawvideo','-pixel_format','rgb24','-video_size',f'{WIDTH}x{HEIGHT}',
            '-framerate',str(fps),'-i','pipe:0','-an','-c:v','libx264','-preset','fast',
            '-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(self.path)],
            stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=self.log,
            creationflags=subprocess.CREATE_NO_WINDOW)

    def add(self,frame):
        if frame.size!=(WIDTH,HEIGHT): raise ValueError('Unexpected frame size')
        self.proc.stdin.write(frame.convert('RGB').tobytes()); self.count+=1

    def close(self):
        if self.proc.stdin and not self.proc.stdin.closed:self.proc.stdin.close()
        code=self.proc.wait(timeout=60); self.log.close()
        if code or self.count==0:raise RuntimeError('Video encoding failed: '+str(self.path))
        return {'file':self.path.name,'frames':self.count,'fps':self.fps,
                'duration_s':self.count/self.fps,'resolution':[WIDTH,HEIGHT]}
