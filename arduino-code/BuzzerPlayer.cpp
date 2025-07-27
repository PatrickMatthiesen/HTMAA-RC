#include "BuzzerPlayer.h"
#include <Arduino.h>

// Static instance for callbacks
BuzzerPlayer *BuzzerPlayer::instance = nullptr;

BuzzerPlayer::BuzzerPlayer(int pin, int channel) : buzzerPin(pin), channel(channel), isPlaying(false), currentNoteIndex(0)
{
    // Configure LEDC for buzzer
    ledcSetup(channel, 1000, 8); // channel, freq, resolution
    ledcAttachPin(buzzerPin, channel);

    // Set static instance for callbacks
    instance = this;
}

BuzzerPlayer::~BuzzerPlayer()
{
    stopTone();
    ledcDetachPin(buzzerPin);
}

void BuzzerPlayer::playTone(int frequency, int duration)
{
    if (frequency > 0)
    {
        ledcWriteTone(channel, frequency);
    }
    else
    {
        ledcWrite(channel, 0); // silence
    }

    noteStartTime = millis();
    noteDuration = duration;
    isPlaying = true;
}

void BuzzerPlayer::stopTone()
{
    ledcWrite(channel, 0);
    isPlaying = false;
    melodyNotes.clear();
    currentNoteIndex = 0;
}

void BuzzerPlayer::playMelody(const std::vector<Note> &notes)
{
    melodyNotes = notes;
    currentNoteIndex = 0;
    if (!melodyNotes.empty())
    {
        playTone(melodyNotes[0].frequency, melodyNotes[0].duration);
    }
}

void BuzzerPlayer::playPresetMelody(PresetMelody melody)
{
    std::vector<Note> notes;

    switch (melody)
    {
    case MELODY_MARIO:
        notes = getMarioMelody();
        break;
    case MELODY_MARIO_COURSE_CLEAR:
        notes = getMarioCourseClearMelody();
        break;
    case MELODY_TETRIS:
        notes = getTetrisMelody();
        break;
    case MELODY_NOKIA:
        notes = getNokiaMelody();
        break;
    case MELODY_HAPPY_BIRTHDAY:
        notes = getHappyBirthdayMelody();
        break;
    case MELODY_STAR_WARS:
        notes = getStarWarsMelody();
        break;
    case MELODY_BEEP_SUCCESS:
        notes = getSuccessBeep();
        break;
    case MELODY_BEEP_ERROR:
        notes = getErrorBeep();
        break;
    case MELODY_NONE:
        stopTone();
        return; // No melody to play
    default:
        return;
    }

    playMelody(notes);
}

void BuzzerPlayer::update()
{
    if (isPlaying && millis() - noteStartTime >= noteDuration)
    {
        if (!melodyNotes.empty())
        {
            // Playing a melody - move to next note
            currentNoteIndex++;
            if (currentNoteIndex < melodyNotes.size())
            {
                playTone(melodyNotes[currentNoteIndex].frequency, melodyNotes[currentNoteIndex].duration);
            }
            else
            {
                // Melody finished
                stopTone();
            }
        }
        else
        {
            // Single tone finished
            stopTone();
        }
    }
}

// Preset melodies
std::vector<Note> BuzzerPlayer::getMarioMelody()
{
    return {
        // Main theme
        {NOTE_E5, 150}, {NOTE_E5, 150}, {NOTE_REST, 150}, {NOTE_E5, 150}, {NOTE_REST, 150}, {NOTE_C5, 150}, {NOTE_E5, 150}, {NOTE_REST, 150}, {NOTE_G5, 150}, {NOTE_REST, 450}, {NOTE_G4, 150}, {NOTE_REST, 450},
        {NOTE_C5, 300}, {NOTE_REST, 150}, {NOTE_G4, 300}, {NOTE_REST, 150}, {NOTE_E4, 300}, {NOTE_REST, 150}, {NOTE_A4, 150}, {NOTE_B4, 150}, {NOTE_AS4, 150}, {NOTE_A4, 300},
        {NOTE_G4, 150}, {NOTE_E5, 150}, {NOTE_G5, 150}, {NOTE_A5, 300}, {NOTE_F5, 150}, {NOTE_G5, 150}, {NOTE_REST, 150}, {NOTE_E5, 300}, {NOTE_C5, 150}, {NOTE_D5, 150}, {NOTE_B4, 300}, {NOTE_REST, 150}
    };
}

std::vector<Note> BuzzerPlayer::getMarioCourseClearMelody()
{
    return {
        {NOTE_G4, 100}, {NOTE_C5, 100}, {NOTE_E5, 100}, {NOTE_G5, 100}, {NOTE_C6, 100}, {NOTE_E6, 100}, {NOTE_G5, 400}};
}

std::vector<Note> BuzzerPlayer::getTetrisMelody()
{
    return {
        {NOTE_E5, 400}, {NOTE_B4, 200}, {NOTE_C5, 200}, {NOTE_D5, 400}, {NOTE_C5, 200}, {NOTE_B4, 200}, {NOTE_A4, 400}, {NOTE_A4, 200}, {NOTE_C5, 200}, {NOTE_E5, 400}, {NOTE_D5, 200}, {NOTE_C5, 200}, {NOTE_B4, 600}, {NOTE_C5, 200}, {NOTE_D5, 400}, {NOTE_E5, 400}, {NOTE_C5, 400}, {NOTE_A4, 400}, {NOTE_A4, 800}};
}

std::vector<Note> BuzzerPlayer::getNokiaMelody()
{
    return {
        {NOTE_E5, 200}, {NOTE_D5, 200}, {NOTE_FS4, 400}, {NOTE_GS4, 400}, {NOTE_CS5, 200}, {NOTE_B4, 200}, {NOTE_D4, 400}, {NOTE_E4, 400}, {NOTE_B4, 200}, {NOTE_A4, 200}, {NOTE_CS4, 400}, {NOTE_E4, 400}, {NOTE_A4, 800}};
}

std::vector<Note> BuzzerPlayer::getHappyBirthdayMelody()
{
    return {
        {NOTE_C4, 250}, {NOTE_C4, 125}, {NOTE_D4, 375}, {NOTE_C4, 375}, {NOTE_F4, 375}, {NOTE_E4, 750}, {NOTE_C4, 250}, {NOTE_C4, 125}, {NOTE_D4, 375}, {NOTE_C4, 375}, {NOTE_G4, 375}, {NOTE_F4, 750}, {NOTE_C4, 250}, {NOTE_C4, 125}, {NOTE_C5, 375}, {NOTE_A4, 375}, {NOTE_F4, 375}, {NOTE_E4, 375}, {NOTE_D4, 750}};
}

std::vector<Note> BuzzerPlayer::getStarWarsMelody()
{
    return {
        {NOTE_A4, 500}, {NOTE_A4, 500}, {NOTE_A4, 500}, {NOTE_F4, 350}, {NOTE_C5, 150}, {NOTE_A4, 500}, {NOTE_F4, 350}, {NOTE_C5, 150}, {NOTE_A4, 650}, {NOTE_REST, 500}, {NOTE_E5, 500}, {NOTE_E5, 500}, {NOTE_E5, 500}, {NOTE_F5, 350}, {NOTE_C5, 150}, {NOTE_GS4, 500}, {NOTE_F4, 350}, {NOTE_C5, 150}, {NOTE_A4, 650}};
}

std::vector<Note> BuzzerPlayer::getSuccessBeep()
{
    return {
        {NOTE_C5, 150}, {NOTE_E5, 150}, {NOTE_G5, 300}};
}

std::vector<Note> BuzzerPlayer::getErrorBeep()
{
    return {
        {NOTE_G4, 200}, {NOTE_D4, 200}, {NOTE_G4, 200}, {NOTE_D4, 200}};
}