"""Bounded future waits and action cleanup for the visible exercise-7 client.

No executor thread is started here. Goal, result and feedback callbacks remain
in the notebook, which uses its IntroLab's context and single executor.
"""
import math
import time
import warnings


def wait_action_future(lab, future, timeout=5.0, description='odpowiedź akcji'):
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('Limit czasu musi być dodatni i skończony.')
    lab.wait_for(future.done, timeout=timeout, description=description)
    return future.result()


def close_action_lab(namespace, timeout=5.0):
    """Cancel tracked active goals before destroying their clients and context.

    The notebook records each send/result future in proby_akcji. Cleanup also
    resolves a pending goal acceptance, so interrupting between send and reply
    does not silently leave an accepted goal executing on a shared simulator.
    """
    lab = namespace.get('lab')
    clients = [namespace.get(name) for name in ('action_client', 'nav_action_client')]
    errors = []
    try:
        if lab is not None and lab.context.ok():
            deadline = time.monotonic() + timeout
            for attempt in namespace.get('proby_akcji', []):
                send = attempt.get('send_future')
                if send is None:
                    continue
                try:
                    handle = wait_action_future(lab, send, max(0.001, deadline - time.monotonic()),
                                                'przyjęcie celu podczas sprzątania')
                    if not handle.accepted:
                        continue
                    result = attempt.get('result_future')
                    if result is None:
                        result = handle.get_result_async()
                    if result.done():
                        result.result()  # Preserve callback/transport exceptions.
                        continue
                    cancel = handle.cancel_goal_async()
                    wait_action_future(lab, cancel, max(0.001, deadline - time.monotonic()),
                                       'odpowiedź na anulowanie podczas sprzątania')
                    wait_action_future(lab, result, max(0.001, deadline - time.monotonic()),
                                       'końcowy wynik podczas sprzątania')
                except Exception as error:
                    errors.append(str(error))
    finally:
        try:
            for client in clients:
                if client is not None:
                    try:
                        client.destroy()
                    except Exception as error:
                        errors.append(str(error))
        finally:
            if lab is not None:
                lab.close()
            namespace['action_client'] = None
            namespace['nav_action_client'] = None
            namespace['proby_akcji'] = []
    if errors:
        warnings.warn('Nie potwierdzono pełnego sprzątania akcji: ' + '; '.join(errors)
                      + '. Timeout nie cofa celu. Sprawdź robota i jego serwer.', RuntimeWarning)
