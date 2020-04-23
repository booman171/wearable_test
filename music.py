#Plays audio files in the current directory
 
import pygame, os.path
from pygame.locals import *
import sys
 
pygame.mixer.init()
musicList={}
x=0
y=70
playing=False
paused=False
change=False
 
pygame.init()
screen = pygame.display.set_mode((320, 240))
pygame.display.set_caption("Ynori7's Media Player")
pygame.mouse.set_visible(1)
background = pygame.Surface(screen.get_size())
background = background.convert()
background.fill((250, 250, 250))
 
def recur(DIR):
    for f in os.listdir(DIR):
        if f[len(f)-4] != '.' and f[len(f)-3] != '.' and f[len(f)-5] != '.':
            recur(DIR+'\\'+f)
        elif f.endswith(".mp3") or f.endswith(".mid"):
            print(f)
            musicList[len(musicList)]=''+DIR+'\\'+f+''
    return musicList
 
musicList=recur(os.curdir)
 
font = pygame.font.Font(None, 36)
text = font.render("Welcome To Ynori7's Media Player", 1, (10, 10, 10))
font = pygame.font.Font(None, 20)
text1 = font.render("down=play/pause  up=stop  right=next  left=previous  q=quit", 1, (10, 10, 10))
background.blit(text, (0, 0))
background.blit(text1, (0, 30))
# Blit everything to the screen
screen.blit(background, (0, 0))
pygame.display.flip()
 
font = pygame.font.Font(None, 20)
text = font.render("Now Playing:", 1, (10, 10, 10))
text1 = font.render("Next Song:", 1, (10, 10, 10))
text2 = font.render("Previous Song:", 1, (10, 10, 10))
background.blit(text, (0, 60))
background.blit(text1, (0, 110))
background.blit(text2, (0, 160))
screen.blit(background, (0, 0))
pygame.display.flip()
 
def next(x, musicList):
        pygame.mixer.music.load(musicList[x])
        pygame.mixer.music.play()
        x+=1
    
def GetSong(musicList, x):
    if not pygame.mixer.music.get_busy():
        pygame.mixer.music.load(musicList[x])
        pygame.mixer.music.play()
    else:
        pygame.mixer.music.stop()
        pygame.mixer.music.load(musicList[x])
        pygame.mixer.music.play()
 
def Stop():
    pygame.mixer.music.stop()
def Pause():
    pygame.mixer.music.pause()
def Resume():
    pygame.mixer.music.unpause()
 
while 1:
    for event in pygame.event.get():
        if event.type == KEYDOWN:
            print(event.key)
            inp=event.key
            if inp==K_DOWN:#play/pause
                if not playing:
                    GetSong(musicList, x)
                    playing=True
                    change=True
                if paused:
                    Resume()
                    paused=False
                else:
                    Pause()
                    paused=True
            if inp==K_UP:#Stop
                Stop()
                playing=False
                change=True
            if inp==K_RIGHT:#Next
                if x<len(musicList)-1:
                    x+=1
                    GetSong(musicList, x)
                    playing=True
                else:
                    x=0
                    GetSong(musicList, x)
                    playing=True
                change=True
            if inp==K_LEFT:#Previous
                if x>0:
                    x-=1
                    GetSong(musicList, x)
                    playing=True
                else:
                    x=len(musicList)-1
                    GetSong(musicList, x)
                    playing=True
                change=True
            if inp==113:#quit
                sys.exit(1)
        elif not pygame.mixer.music.get_busy() and playing:
            if x<len(musicList)-1:
                x+=1
                GetSong(musicList, x)
                change=True
            else:
                playing=False
                x=0
        if change:
            background.fill((250, 250, 250), (0, 80, 400, 20))
            background.fill((250, 250, 250), (0, 130, 400, 20))
            background.fill((250, 250, 250), (0, 180, 400, 20))
            if playing:
                font = pygame.font.Font(None, 18)
                text = font.render(musicList[x], 1, (10, 10, 10))#current
                if x>0:#previous
                    text1 = font.render(musicList[x-1], 1, (10, 10, 10))
                else:
                    text1 = font.render(musicList[len(musicList)-1], 1, (10, 10, 10))
                if x<len(musicList)-1:#next
                    text2 = font.render(musicList[x+1], 1, (10, 10, 10))
                else:
                    text2 = font.render(musicList[0], 1, (10, 10, 10))
                background.blit(text, (0, 80))
                background.blit(text2, (0, 130))
                background.blit(text1, (0, 180))
            screen.blit(background, (0, 0))
            pygame.display.flip()
        change=False
