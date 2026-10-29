/*
 * scheduler.c — ESQUELETO DEL LABORATORIO
 *
 * Este archivo contiene el núcleo del scheduler round-robin de miniOS.
 * Las funciones de infraestructura (init, getters, install_sigchld, stop,
 * timespec_diff_ms) ya están implementadas.
 *
 * Tu trabajo es implementar las CUATRO funciones marcadas con [TODO]:
 *   1. scheduler_create_process  — fork + exec + SIGSTOP + PCB init
 *   2. scheduler_start           — arrancar el primer proceso y el timer
 *   3. scheduler_tick            — handler de SIGALRM (context switch)
 *   4. scheduler_sigchld         — handler de SIGCHLD (terminación)
 *
 * Cada función viene con comentarios numerados que describen el flujo
 * paso a paso. Tu trabajo es traducir cada paso a código C usando las
 * APIs de POSIX y las funciones de infraestructura disponibles.
 *
 * APIs disponibles:
 *   - POSIX:       fork, execl, waitpid, kill, clock_gettime
 *   - platform_*:  ver src/platform/platform.h
 *   - pcb_*:       ver src/pcb.h
 *   - rq_*:        ver src/ready_queue.h
 *   - timer_*:     ver src/timer.h
 *   - monitor_*:   ver src/monitor.h
 *
 * REGLAS DE SEGURIDAD EN SEÑALES (importantes para scheduler_tick y
 * scheduler_sigchld):
 *   - NO uses printf/fprintf dentro de los handlers (no son
 *     async-signal-safe). Solo kill, waitpid, clock_gettime, write.
 *   - El shell bloquea SIGALRM con sigprocmask durante sus operaciones
 *     críticas, por lo que no necesitas mutex manual sobre process_table.
 */

#include "scheduler.h"
#include "timer.h"
#include "monitor.h"
#include "platform/platform.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <libgen.h>

// Estado global del scheduler
static volatile int current_running = -1;   // índice en process_table del proceso RUNNING, -1 si ninguno
static volatile int scheduler_active = 0;   // 1 si el scheduler está corriendo

// ============================================================
// Helpers ya implementados — NO los modifiques
// ============================================================

double timespec_diff_ms(struct timespec end, struct timespec start) {
    double sec = (double)(end.tv_sec - start.tv_sec);
    double nsec = (double)(end.tv_nsec - start.tv_nsec);
    return sec * 1000.0 + nsec / 1000000.0;
}

void scheduler_init(void) {
    process_count = 0;
    current_running = -1;
    scheduler_active = 0;
    rq_init();
}

int scheduler_get_running(void) {
    return current_running;
}

int scheduler_is_running(void) {
    return scheduler_active;
}

void scheduler_install_sigchld(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = scheduler_sigchld;
    sa.sa_flags = SA_RESTART | SA_NOCLDSTOP;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGCHLD, &sa, NULL);
}

void scheduler_stop(void) {
    timer_stop();
    scheduler_active = 0;

    for (int i = 0; i < process_count; i++) {
        if (process_table[i].state != PROC_TERMINATED) {
            kill(process_table[i].pid, SIGKILL);
            int status;
            waitpid(process_table[i].pid, &status, 0);
            process_table[i].state = PROC_TERMINATED;
        }
    }
    current_running = -1;
}


// ============================================================
// [TODO 1/4] scheduler_create_process
// ------------------------------------------------------------
// Crea un proceso nuevo a partir de un binario, lo deja detenido
// con estado PROC_READY y lo encola en la ready queue.
//
// Retorna el índice del nuevo PCB en process_table, o -1 en error.
//
// Observable correcto: `ps aux | grep <binario>` debe mostrar el
// proceso en estado T (stopped) justo después de crearlo.
// ============================================================
int scheduler_create_process(const char *path, const char *arg) {
    if (process_count >= MAX_PROCESSES) {
        fprintf(stderr, "Error: tabla de procesos llena (maximo %d).\n", MAX_PROCESSES);
        return -1;
    }

    /* Evitar que SIGCHLD recoja el hijo antes de registrarlo en la tabla. */
    sigset_t chld_mask, previous_mask;
    sigemptyset(&chld_mask);
    sigaddset(&chld_mask, SIGCHLD);
    sigprocmask(SIG_BLOCK, &chld_mask, &previous_mask);

    pid_t pid = fork();
    if (pid < 0) {
        perror("fork");
        sigprocmask(SIG_SETMASK, &previous_mask, NULL);
        return -1;
    }
    if (pid == 0) {
        /* El hijo no debe heredar SIGCHLD bloqueada durante exec. */
        sigprocmask(SIG_SETMASK, &previous_mask, NULL);
        if (platform_uses_ptrace() && platform_trace_child() < 0) {
            perror("ptrace");
            _exit(1);
        }
        if (arg != NULL) execl(path, path, arg, (char *)NULL);
        else execl(path, path, (char *)NULL);
        perror("execl");
        _exit(127);
    }

    int status = 0;
    if (platform_uses_ptrace()) {
        if (waitpid(pid, &status, 0) != pid || !WIFSTOPPED(status)) {
            fprintf(stderr, "Error: hijo no llego al stop post-exec.\n");
            goto fail;
        }
    }

    int idx = process_count;
    char *path_copy = strdup(path);
    if (!path_copy) {
        perror("strdup");
        goto fail;
    }
    pcb_init(&process_table[idx], pid, basename(path_copy));
    free(path_copy);

    if (platform_uses_ptrace()) {
        if (platform_get_registers(pid, &process_table[idx].registers) == 0)
            process_table[idx].regs_valid = 1;
        if (platform_detach(pid) < 0) {
            perror("ptrace detach");
            goto fail;
        }
    }

    if (platform_stop_process(pid) < 0) {
        perror("SIGSTOP");
        goto fail;
    }
    if (waitpid(pid, &status, WUNTRACED) != pid || !WIFSTOPPED(status)) {
        fprintf(stderr, "Error: el hijo no quedo detenido en READY.\n");
        goto fail;
    }

    process_table[idx].state = PROC_READY;
    if (rq_enqueue(idx) < 0) {
        fprintf(stderr, "Error: ready queue llena.\n");
        goto fail;
    }
    process_count++;
    monitor_emit_created(pid, process_table[idx].name);
    if (process_table[idx].regs_valid) {
        monitor_emit_registers(pid,
            (unsigned long long)process_table[idx].registers.program_counter,
            (unsigned long long)process_table[idx].registers.stack_pointer);
    }
    sigprocmask(SIG_SETMASK, &previous_mask, NULL);
    return idx;

fail:
    kill(pid, SIGKILL);
    waitpid(pid, &status, 0);
    sigprocmask(SIG_SETMASK, &previous_mask, NULL);
    return -1;
}


// ============================================================
// [TODO 2/4] scheduler_start
// ------------------------------------------------------------
// Arranca el scheduler: desencola el primer proceso, lo pone en
// RUNNING, le manda SIGCONT e instala el timer que dispara el
// context switch periódicamente.
//
// Observable correcto: tras crear 1 proceso y llamar start, ese
// proceso empieza a producir salida en la terminal.
// ============================================================
void scheduler_start(int slice_ms) {
    if (rq_is_empty()) {
        printf("No hay procesos en la ready queue.\n");
        return;
    }
    int idx = rq_dequeue();
    if (idx < 0) return;

    pcb_t *next = &process_table[idx];
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    next->state = PROC_RUNNING;
    next->last_started = now;
    next->wait_time_ms = timespec_diff_ms(now, next->created_at);
    current_running = idx;
    scheduler_active = 1;
    timer_init(slice_ms, scheduler_tick);
    timer_start();
    if (platform_resume_process(next->pid) < 0)
        kill(next->pid, SIGKILL);
}


// ============================================================
// [TODO 3/4] scheduler_tick  — handler de SIGALRM
// ------------------------------------------------------------
// Se invoca CADA vez que expira el time slice. Realiza el
// context switch: detiene al proceso actual, actualiza su PCB,
// lo manda al final de la cola, saca al siguiente y lo reanuda.
//
// ¡IMPORTANTE! Esta función corre en un signal handler.
//   - NO llames printf/fprintf/malloc.
//   - Solo kill, waitpid, clock_gettime, write son seguros.
//   - Las funciones monitor_emit_* internamente usan snprintf + write,
//     que es aceptable para este proyecto educativo.
//
// Observable correcto: el Gantt chart del dashboard muestra
// segmentos alternados entre procesos cada ~slice_ms.
// ============================================================
void scheduler_tick(int signum) {
    (void)signum;
    if (current_running < 0 || !scheduler_active) return;

    /* Evita carreras entre SIGALRM (rotacion) y SIGCHLD (terminacion). */
    sigset_t mask, oldmask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGCHLD);
    sigprocmask(SIG_BLOCK, &mask, &oldmask);

    pcb_t *current = &process_table[current_running];
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    /* Con un solo proceso no se necesita cambio de contexto real. */
    if (rq_is_empty()) {
        current->cpu_time_ms += timespec_diff_ms(now, current->last_started);
        current->last_started = now;
        sigprocmask(SIG_SETMASK, &oldmask, NULL);
        return;
    }

    int previous_idx = current_running;
    pid_t from_pid = current->pid;
    platform_stop_process(from_pid);
    current->cpu_time_ms += timespec_diff_ms(now, current->last_started);
    current->state = PROC_READY;
    current->context_switches++;
    rq_enqueue(previous_idx);

    int next_idx = rq_dequeue();
    if (next_idx < 0) {
        current_running = -1;
        scheduler_active = 0;
        timer_stop();
        sigprocmask(SIG_SETMASK, &oldmask, NULL);
        return;
    }
    pcb_t *next = &process_table[next_idx];
    clock_gettime(CLOCK_MONOTONIC, &now);
    next->state = PROC_RUNNING;
    next->last_started = now;
    double wait_ms = timespec_diff_ms(now, next->created_at) - next->cpu_time_ms;
    next->wait_time_ms = wait_ms > 0 ? wait_ms : 0;
    current_running = next_idx;
    platform_resume_process(next->pid);
    monitor_emit_switch(from_pid, next->pid, timer_get_slice());
    sigprocmask(SIG_SETMASK, &oldmask, NULL);
}


// ============================================================
// [TODO 4/4] scheduler_sigchld  — handler de SIGCHLD
// ------------------------------------------------------------
// Se invoca cuando un proceso hijo termina. Debe:
//   - Detectar TODOS los hijos terminados (puede haber varios).
//   - Actualizar su PCB a PROC_TERMINATED.
//   - Si el proceso que terminó era el RUNNING, despachar al siguiente.
//   - Si estaba en la ready queue, removerlo con rq_remove().
//
// Observable correcto: tras `exit` en miniOS, `ps aux | grep defunct`
// no muestra zombies.
// ============================================================
void scheduler_sigchld(int signum) {
    (void)signum;
    /* Evita que SIGALRM interrumpa la limpieza de tabla y cola. */
    sigset_t mask, oldmask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGALRM);
    sigprocmask(SIG_BLOCK, &mask, &oldmask);

    int status;
    pid_t pid;
    while ((pid = waitpid(-1, &status, WNOHANG | WUNTRACED)) > 0) {
        if (!WIFEXITED(status) && !WIFSIGNALED(status)) continue;
        for (int i = 0; i < process_count; i++) {
            pcb_t *pcb = &process_table[i];
            if (pcb->pid != pid || pcb->state == PROC_TERMINATED) continue;

            int was_running = (i == current_running);
            if (was_running) {
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                pcb->cpu_time_ms += timespec_diff_ms(now, pcb->last_started);
                current_running = -1;
            } else {
                rq_remove(i);
            }
            pcb->state = PROC_TERMINATED;
            monitor_emit_terminated(pid, pcb->cpu_time_ms, pcb->context_switches);

            if (was_running) {
                int next_idx = rq_dequeue();
                if (next_idx >= 0) {
                    pcb_t *next = &process_table[next_idx];
                    struct timespec now;
                    clock_gettime(CLOCK_MONOTONIC, &now);
                    next->state = PROC_RUNNING;
                    next->last_started = now;
                    double wait_ms = timespec_diff_ms(now, next->created_at) - next->cpu_time_ms;
                    next->wait_time_ms = wait_ms > 0 ? wait_ms : 0;
                    current_running = next_idx;
                    platform_resume_process(next->pid);
                    monitor_emit_switch(pid, next->pid, timer_get_slice());
                } else {
                    scheduler_active = 0;
                    timer_stop();
                }
            }
            break;
        }
    }
    sigprocmask(SIG_SETMASK, &oldmask, NULL);
}
