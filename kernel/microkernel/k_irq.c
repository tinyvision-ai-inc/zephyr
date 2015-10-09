/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @brief Task IRQ kernel services
 *
 * This module manages the interrupt functionality between a task level device
 * driver and the kernel.
 *
 * A task level device driver allocates a specific task IRQ object by providing
 * an IRQ and priority level; the registration process allocates an interrupt
 * vector, sets up an ISR, and enables the associated interrupt. When an
 * interrupt occurs, the ISR handler signals an event specific to the IRQ object.
 * The task level device driver tests/waits on this event number to determine
 * if/when the interrupt has occurred. As the ISR also disables the interrupt, the
 * task level device driver subsequently make a request to have the interrupt
 * enabled again. If desired, the device driver can free an IRQ object that
 * it is no longer interested in using.
 *
 * These routines perform error checking to ensure that an IRQ object can only be
 * allocated by a single task, and that subsequent operations on that IRQ object
 * are only performed by that task. This checking is necessary to ensure that
 * a task cannot impact the operation of an IRQ object it does not own.
 *
 */

#if (CONFIG_MAX_NUM_TASK_IRQS > 0)

#include <nano_private.h>
#include <microkernel.h>
#include <micro_private.h>
#include <misc/__assert.h>

#define MAX_TASK_IRQS CONFIG_MAX_NUM_TASK_IRQS

#define INVALID_TASK -1

/* task IRQ object registration request */

struct irq_obj_reg_arg {
	kirq_t irq_obj;   /* IRQ object identifier */
	uint32_t irq;  /* IRQ of device */
	ktask_t task_id; /* requesting task */
};

/* task IRQ object type */

struct task_irq_info {
	ktask_t task_id;  /* task ID of task IRQ object's owner */
	uint32_t irq;    /* IRQ used by task IRQ object */
	kevent_t event;  /* event number assigned to task IRQ object */
	uint32_t vector; /* interrupt vector assigned to task IRQ object */
};

/* task IRQ object array */

static struct task_irq_info task_irq_object[MAX_TASK_IRQS] = {
	[0 ...(MAX_TASK_IRQS - 1)].task_id = INVALID_TASK
};

/* architecture-specific */

#if defined(CONFIG_X86)

#define RELEASE_VECTOR(v) _IntVecMarkFree(v)

#elif defined(CONFIG_CPU_CORTEX_M3_M4)
#include <arch/cpu.h>
extern void _irq_disconnect(unsigned int irq);
#define RELEASE_VECTOR(v) _irq_disconnect(v)
#else
#error "Unknown target"
#endif

/* array of event id used by task IRQ objects */
extern const kevent_t _TaskIrqEvt_objIds[];

/**
 *
 * @brief ISR for task IRQ objects
 *
 * This ISR handles interrupts generated by registered task IRQ objects.
 *
 * The ISR triggers an event signal specified by the event number associated
 * with a particular task IRQ object; the interrupt for the task IRQ object
 * is then disabled. The parameter provided to the ISR is a structure that
 * contains information about the objects's vector, IRQ, and event number.
 *
 * This ISR does not facilitate an int acknowledgment as it presumes that an
 * End of Interrupt (EOI) routine is provided by the interrupt controller that
 * is being used.
 *
 * @param parameter Pointer to task IRQ object
 *
 * @return N/A
 */
static void task_irq_int_handler(void *parameter)
{
	struct task_irq_info *irq_obj_ptr = parameter;

	isr_event_send(irq_obj_ptr->event);
	irq_disable(irq_obj_ptr->irq);
}

void task_irq_free(kirq_t irq_obj)
{
	__ASSERT(irq_obj < MAX_TASK_IRQS, "Invalid IRQ object");
	__ASSERT(task_irq_object[irq_obj].task_id == task_id_get(),
			 "Incorrect Task ID");

	irq_disable(task_irq_object[irq_obj].irq);
	RELEASE_VECTOR(task_irq_object[irq_obj].vector);
	(void)task_event_recv(task_irq_object[irq_obj].event);
	task_irq_object[irq_obj].task_id = INVALID_TASK;
}

/**
 *
 * @brief Re-enable a task IRQ object's interrupt
 *
 * This re-enables the interrupt for a task IRQ object.
 * @param irq_obj IRQ object identifier
 *
 * @return N/A
 */
void task_irq_ack(kirq_t irq_obj)
{
	__ASSERT(irq_obj < MAX_TASK_IRQS, "Invalid IRQ object");
	__ASSERT(task_irq_object[irq_obj].task_id == task_id_get(),
			 "Incorrect Task ID");

	irq_enable(task_irq_object[irq_obj].irq);
}

/**
 *
 * @brief Determine if a task IRQ object has had an interrupt
 *
 * This tests a task IRQ object to see if it has signaled an interrupt.
 * @param irq_obj IRQ object identifier
 * @param time  Time to wait (in ticks)
 *
 * @return RC_OK, RC_FAIL, or RC_TIME
 */
int _task_irq_test(kirq_t irq_obj, int32_t time)
{
	__ASSERT(irq_obj < MAX_TASK_IRQS, "Invalid IRQ object");
	__ASSERT(task_irq_object[irq_obj].task_id == task_id_get(),
			 "Incorrect Task ID");

	return _task_event_recv(task_irq_object[irq_obj].event, time);
}

/**
 *
 * @brief Allocate a task IRQ object
 *
 * This routine allocates a task IRQ object to a task.
 * @param arg Pointer to registration request arguments
 *
 * @return ptr to allocated task IRQ object if successful, NULL if not
 */
static int _k_task_irq_alloc(void *arg)
{
	struct irq_obj_reg_arg *argp = (struct irq_obj_reg_arg *)arg;
	struct task_irq_info *irq_obj_ptr; /* ptr to task IRQ object */
	int curr_irq_obj;	/* IRQ object loop counter */

	/* Fail if the requested IRQ object is already in use */

	if (task_irq_object[argp->irq_obj].task_id != INVALID_TASK) {
		return (int)NULL;
	}

	/* Fail if the requested IRQ is already in use */

	for (curr_irq_obj = 0; curr_irq_obj < MAX_TASK_IRQS; curr_irq_obj++) {
		if ((task_irq_object[curr_irq_obj].irq == argp->irq) &&
		    (task_irq_object[curr_irq_obj].task_id != INVALID_TASK)) {
			return (int)NULL;
		}
	}

	/* Take ownership of specified IRQ object */

	irq_obj_ptr = &task_irq_object[argp->irq_obj];
	irq_obj_ptr->task_id = argp->task_id;
	irq_obj_ptr->irq = argp->irq;
	irq_obj_ptr->event = _TaskIrqEvt_objIds[argp->irq_obj];
	irq_obj_ptr->vector = INVALID_VECTOR;

	return (int)irq_obj_ptr;
}


uint32_t task_irq_alloc(kirq_t irq_obj,	uint32_t irq, uint32_t priority,
			uint32_t flags)
{
	struct irq_obj_reg_arg arg;  /* IRQ object registration request arguments */
	struct task_irq_info *irq_obj_ptr; /* ptr to task IRQ object */

	/* Allocate the desired IRQ object and IRQ */

	arg.irq_obj = irq_obj;
	arg.irq = irq;
	arg.task_id = task_id_get();

	irq_obj_ptr = (struct task_irq_info *)task_offload_to_fiber(_k_task_irq_alloc,
						     (void *)&arg);
	if (irq_obj_ptr == NULL) {
		return INVALID_VECTOR;
	}

	/*
	 * NOTE: the comma that seems to be missing is part of the IRQ_STUB
	 *       definition to abstract the different irq_connect signatures
	 */
	irq_obj_ptr->vector = irq_connect(
		irq, priority, task_irq_int_handler, (void *)irq_obj_ptr,
		flags);
	irq_enable(irq);

	return irq_obj_ptr->vector;
}


#endif /* (CONFIG_MAX_NUM_TASK_IRQS > 0) */
