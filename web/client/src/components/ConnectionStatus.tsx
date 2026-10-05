import type { ConnectionStatus as Status } from '../types/simulation';
export function ConnectionStatus({ status }: { status: Status }) {
  return <span className={`connection ${status}`} role="status"><i />{status}</span>;
}
